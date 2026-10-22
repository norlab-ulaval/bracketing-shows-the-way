#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <basler_camera/msg/basler_frame_meta.hpp>

#include "BracketingDriver.h"
#include "Config.h"

#include <pylon/PylonIncludes.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using Image    = sensor_msgs::msg::Image;
using ImagePub = rclcpp::Publisher<Image>::SharedPtr;
using Meta     = basler_camera::msg::BaslerFrameMeta;
using MetaPub  = rclcpp::Publisher<Meta>::SharedPtr;

class BaslerCameraNode : public rclcpp::Node {
public:
    BaslerCameraNode()
    : Node("basler_camera")
    {
        declareParameters();
        cfg_ = buildConfig();
        camera1_frame_id_ = get_parameter("camera_1_frame_id").as_string();
        camera2_frame_id_ = get_parameter("camera_2_frame_id").as_string();
        width_  = cfg_.width;
        height_ = cfg_.height;
        // A smaller width/height crops a centered AOI; the format stays BayerRG8.
        image_encoding_ = "bayer_rggb8";

        Pylon::PylonInitialize();
        try {
            if (cfg_.camera1SerialNumber.empty())
                autoDetectCameras();
            mono_ = cfg_.camera2SerialNumber.empty();
            if (mono_)
                RCLCPP_INFO(get_logger(), "Mono mode: camera_1 = %s only (no stereo, camera_2 "
                            "topics not advertised)", cfg_.camera1SerialNumber.c_str());
            createPublishers();
            driver_ = std::make_unique<BracketingDriver>(cfg_);

            driver_->setCamera1Callback([this](const CapturedFrame& f) {
                publishFrame(f,
                             camera1_raw_, camera1_zone_pubs_,
                             camera1_meta_, camera1_frame_id_,
                             camera1_img_, camera1_meta_msg_);
            });
            if (!mono_) {
                driver_->setCamera2Callback([this](const CapturedFrame& f) {
                    publishFrame(f,
                                 camera2_raw_, camera2_zone_pubs_,
                                 camera2_meta_, camera2_frame_id_,
                                 camera2_img_, camera2_meta_msg_);
                });
            }

            capture_thread_ = std::thread([this]() { captureLoop(); });
        } catch (...) {
            driver_.reset();
            Pylon::PylonTerminate();
            throw;
        }
        RCLCPP_INFO(get_logger(), "Basler camera node started");
    }

    /// True once the capture loop has died on an exception.
    bool failed() const { return failed_.load(); }

    ~BaslerCameraNode() override
    {
        running_.store(false);
        if (capture_thread_.joinable())
            capture_thread_.join();
        driver_.reset();
        Pylon::PylonTerminate();
    }

private:
    // ── Parameters ──────────────────────────────────────────────────────────

    void declareParameters()
    {
        // Serials may be integers (unquoted YAML) or strings; accept both.
        rcl_interfaces::msg::ParameterDescriptor dyn;
        dyn.dynamic_typing = true;
        declare_parameter("camera_1_serial", rclcpp::ParameterValue{}, dyn);
        declare_parameter("camera_2_serial", rclcpp::ParameterValue{}, dyn);

        // B-Adapt. See DriverConfig (Config.h) and README.md.
        declare_parameter("ae_zone_count", 3);  // must be 3
        declare_parameter("zone_sort_stride", 8);
        declare_parameter("ae_bridge_low_percentile",  99.9);
        declare_parameter("ae_bridge_low_target",      250.0);
        declare_parameter("ae_bridge_high_percentile", 0.1);
        declare_parameter("ae_bridge_high_target",     5.0);
        declare_parameter("ae_bridge_min_stops_gap",   1.0);

        // Controller tuning.
        declare_parameter("pixel_max",                255.0);
        declare_parameter("ae_zone_alpha", std::vector<double>{0.4, 0.4, 0.4});  // EMA rate: low, mid (unused), high
        declare_parameter("epsilon",                  0.05);
        declare_parameter("default_exposure_us",      10000.0);
        declare_parameter("ae_saturation_boost_fraction", 0.5);
        declare_parameter("exposure_tie_tolerance_us", 5.0);

        declare_parameter("min_exposure_us", 100.0);
        declare_parameter("max_exposure_us", 15000.0);
        declare_parameter("min_gain_db",     0.0);
        declare_parameter("max_gain_db",     12.0);

        // GigE transport, per camera.
        declare_parameter("camera_1_heartbeat_timeout_ms", 5000);
        declare_parameter("camera_2_heartbeat_timeout_ms", 5000);
        declare_parameter("camera_1_packet_size_bytes",    1440);
        declare_parameter("camera_2_packet_size_bytes",    1440);
        declare_parameter("camera_1_inter_packet_delay_ticks", 0);
        declare_parameter("camera_2_inter_packet_delay_ticks", 0);
        declare_parameter("camera_1_bandwidth_reserve_mode", "");
        declare_parameter("camera_2_bandwidth_reserve_mode", "");
        declare_parameter("camera_1_num_grab_buffers", 10);
        declare_parameter("camera_2_num_grab_buffers", 10);
        declare_parameter("camera_1_frame_retention_ms", -1);
        declare_parameter("camera_2_frame_retention_ms", -1);

        // Host-side trigger pacing (Hz), 0 = as fast as possible.
        declare_parameter("acquisition_frame_rate", 0.0);

        declare_parameter("ae_once_max_iters",  100);
        declare_parameter("ae_once_timeout_ms", 5000);
        declare_parameter("ae_once_poll_ms",    5);

        declare_parameter("grab_timeout_ms",          5000);
        declare_parameter("camera_2_grab_timeout_ms", 5000);

        declare_parameter("trigger_output_line",   "Line3");
        declare_parameter("trigger_output_source", "ExposureActive");
        declare_parameter("trigger_input_line",    "Line3");
        declare_parameter("trigger_source",        "Line3");
        declare_parameter("trigger_activation",    "RisingEdge");

        declare_parameter("camera_1_frame_id", "camera_1");
        declare_parameter("camera_2_frame_id", "camera_2");

        declare_parameter("camera_1_wb_red",   1.0);
        declare_parameter("camera_1_wb_green", 1.0);
        declare_parameter("camera_1_wb_blue",  1.0);
        declare_parameter("camera_2_wb_red",   1.0);
        declare_parameter("camera_2_wb_green", 1.0);
        declare_parameter("camera_2_wb_blue",  1.0);

        declare_parameter("width",    1920);
        declare_parameter("height",   1200);

        // Per-frame AE log on stdout (no effect on control).
        declare_parameter("verbose", false);
    }

    // Serial numbers may be stored as integers (unquoted YAML) or strings
    // (quoted YAML / command-line). Handle both so either style works.
    std::string getSerialParam(const std::string& name)
    {
        const auto& p = get_parameter(name);
        switch (p.get_type()) {
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                return std::to_string(p.as_int());
            case rclcpp::ParameterType::PARAMETER_STRING:
                return p.as_string();
            default:
                return "";
        }
    }

    DriverConfig buildConfig()
    {
        DriverConfig cfg;
        cfg.camera1SerialNumber = getSerialParam("camera_1_serial");
        cfg.camera2SerialNumber = getSerialParam("camera_2_serial");

        cfg.aeZoneCount        = get_parameter("ae_zone_count").as_int();
        cfg.zoneSortStride     = get_parameter("zone_sort_stride").as_int();
        cfg.aeBridgeLowPercentile  = get_parameter("ae_bridge_low_percentile").as_double();
        cfg.aeBridgeLowTarget      = get_parameter("ae_bridge_low_target").as_double();
        cfg.aeBridgeHighPercentile = get_parameter("ae_bridge_high_percentile").as_double();
        cfg.aeBridgeHighTarget     = get_parameter("ae_bridge_high_target").as_double();
        cfg.aeBridgeMinStopsGap    = get_parameter("ae_bridge_min_stops_gap").as_double();

        cfg.pixelMax               = get_parameter("pixel_max").as_double();
        cfg.aeZoneAlpha            = get_parameter("ae_zone_alpha").as_double_array();
        cfg.epsilon                = get_parameter("epsilon").as_double();
        cfg.defaultExposureUs      = get_parameter("default_exposure_us").as_double();
        cfg.aeSaturationBoostFraction = get_parameter("ae_saturation_boost_fraction").as_double();
        cfg.exposureTieToleranceUs = get_parameter("exposure_tie_tolerance_us").as_double();

        cfg.minExposureUs = get_parameter("min_exposure_us").as_double();
        cfg.maxExposureUs = get_parameter("max_exposure_us").as_double();
        cfg.minGainDb     = get_parameter("min_gain_db").as_double();
        cfg.maxGainDb     = get_parameter("max_gain_db").as_double();

        cfg.camera1HeartbeatTimeoutMs     = get_parameter("camera_1_heartbeat_timeout_ms").as_int();
        cfg.camera2HeartbeatTimeoutMs     = get_parameter("camera_2_heartbeat_timeout_ms").as_int();
        cfg.camera1PacketSizeBytes        = get_parameter("camera_1_packet_size_bytes").as_int();
        cfg.camera2PacketSizeBytes        = get_parameter("camera_2_packet_size_bytes").as_int();
        cfg.camera1InterPacketDelayTicks  = get_parameter("camera_1_inter_packet_delay_ticks").as_int();
        cfg.camera2InterPacketDelayTicks  = get_parameter("camera_2_inter_packet_delay_ticks").as_int();
        cfg.camera1BandwidthReserveMode   = get_parameter("camera_1_bandwidth_reserve_mode").as_string();
        cfg.camera2BandwidthReserveMode   = get_parameter("camera_2_bandwidth_reserve_mode").as_string();
        cfg.camera1NumGrabBuffers         = get_parameter("camera_1_num_grab_buffers").as_int();
        cfg.camera2NumGrabBuffers         = get_parameter("camera_2_num_grab_buffers").as_int();
        cfg.camera1FrameRetentionMs       = get_parameter("camera_1_frame_retention_ms").as_int();
        cfg.camera2FrameRetentionMs       = get_parameter("camera_2_frame_retention_ms").as_int();

        cfg.acquisitionFrameRate = get_parameter("acquisition_frame_rate").as_double();

        cfg.aeOnceMaxIters  = get_parameter("ae_once_max_iters").as_int();
        cfg.aeOnceTimeoutMs = get_parameter("ae_once_timeout_ms").as_int();
        cfg.aeOncePollMs    = get_parameter("ae_once_poll_ms").as_int();

        cfg.grabTimeoutMs       = get_parameter("grab_timeout_ms").as_int();
        cfg.camera2GrabTimeoutMs = get_parameter("camera_2_grab_timeout_ms").as_int();

        cfg.triggerOutputLine   = get_parameter("trigger_output_line").as_string();
        cfg.triggerOutputSource = get_parameter("trigger_output_source").as_string();
        cfg.triggerInputLine    = get_parameter("trigger_input_line").as_string();
        cfg.triggerSource       = get_parameter("trigger_source").as_string();
        cfg.triggerActivation   = get_parameter("trigger_activation").as_string();

        cfg.width   = get_parameter("width").as_int();
        cfg.height  = get_parameter("height").as_int();

        cfg.camera1WbRed   = get_parameter("camera_1_wb_red").as_double();
        cfg.camera1WbGreen = get_parameter("camera_1_wb_green").as_double();
        cfg.camera1WbBlue  = get_parameter("camera_1_wb_blue").as_double();
        cfg.camera2WbRed   = get_parameter("camera_2_wb_red").as_double();
        cfg.camera2WbGreen = get_parameter("camera_2_wb_green").as_double();
        cfg.camera2WbBlue  = get_parameter("camera_2_wb_blue").as_double();

        cfg.verbose = get_parameter("verbose").as_bool();

        // Fail before pylon is initialized.
        validate(cfg);
        return cfg;
    }

    // camera_1_serial / camera_2_serial unset: one camera on the network =
    // mono with it; two = stereo, lowest serial = camera_1 (left, trigger
    // master); anything else is an error.
    void autoDetectCameras()
    {
        const auto devices = CameraGate::enumerateDevices();
        if (devices.size() == 1) {
            cfg_.camera1SerialNumber = devices[0].serial;
            RCLCPP_INFO(get_logger(), "Auto-detected 1 camera: camera_1 = %s [%s, %s]",
                        devices[0].serial.c_str(), devices[0].model.c_str(),
                        devices[0].ip.c_str());
            return;
        }
        if (devices.size() != 2) {
            std::string found;
            for (const auto& d : devices)
                found += "\n  serial " + d.serial + "  model " + d.model +
                         "  IP " + (d.ip.empty() ? "-" : d.ip);
            throw std::runtime_error(
                "camera auto-detection: found " + std::to_string(devices.size()) +
                " Basler camera(s), need 1 (mono) or 2 (stereo)" + found +
                "\nCheck power, cabling and the GigE NIC configuration (docker: --network host), "
                "or set camera_1_serial:=<left> [camera_2_serial:=<right>] explicitly.");
        }
        cfg_.camera1SerialNumber = devices[0].serial;
        cfg_.camera2SerialNumber = devices[1].serial;
        RCLCPP_INFO(get_logger(),
                    "Auto-detected 2 cameras (sorted by serial): camera_1 (left, trigger master) = %s "
                    "[%s, %s], camera_2 (right, hardware-triggered) = %s [%s, %s]. If left/right are "
                    "swapped, set camera_1_serial:=%s camera_2_serial:=%s.",
                    devices[0].serial.c_str(), devices[0].model.c_str(), devices[0].ip.c_str(),
                    devices[1].serial.c_str(), devices[1].model.c_str(), devices[1].ip.c_str(),
                    devices[1].serial.c_str(), devices[0].serial.c_str());
    }

    // ── Publishers ──────────────────────────────────────────────────────────

    // Mono: camera_1 topics only.
    void createPublishers()
    {
        // Default (reliable) QoS, depth 4.
        constexpr int QD = 4;
        camera1_raw_  = create_publisher<Image>("basler/camera_1/image_raw",  QD);
        camera1_meta_ = create_publisher<Meta> ("basler/camera_1/image_meta", QD);
        if (!mono_) {
            camera2_raw_  = create_publisher<Image>("basler/camera_2/image_raw",  QD);
            camera2_meta_ = create_publisher<Meta> ("basler/camera_2/image_meta", QD);
        }

        // One image_raw topic per zone (zone_0 = low, the darkest bracket).
        for (int zone = 0; zone < cfg_.aeZoneCount; ++zone) {
            const std::string suffix = "/zone_" + std::to_string(zone) + "/image_raw";
            camera1_zone_pubs_.push_back(create_publisher<Image>("basler/camera_1" + suffix, QD));
            if (!mono_)
                camera2_zone_pubs_.push_back(create_publisher<Image>("basler/camera_2" + suffix, QD));
        }
    }

    // ── Capture loop (runs in capture_thread_) ───────────────────────────────

    // A fatal error (camera not found, link lost, trigger-ready timeout)
    // shuts the node down with a non-zero exit code so the launch file can
    // respawn it, instead of leaving a node that spins without publishing.
    void captureLoop()
    {
        try {
            driver_->run(running_);
        } catch (const Pylon::GenericException& e) {
            RCLCPP_FATAL(get_logger(), "Pylon error in capture loop: %s", e.GetDescription());
            failed_.store(true);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Error in capture loop: %s", e.what());
            failed_.store(true);
        }
        running_.store(false);
        if (failed_.load()) {
            RCLCPP_FATAL(get_logger(),
                         "Shutting down. Check that the camera(s) are powered and reachable "
                         "(GigE NIC/IP, docker --network host) and that camera_1_serial / "
                         "camera_2_serial match the camera bodies.");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(get_logger(), "Capture loop exited");
        }
    }

    // ── Frame publishing (called from capture_thread_ via callback) ──────────

    void publishFrame(const CapturedFrame& f,
                      ImagePub& raw, std::vector<ImagePub>& zone_pubs,
                      MetaPub& meta_pub,
                      const std::string& frame_id,
                      Image& img, Meta& meta)
    {
        auto stamp = rclcpp::Time(f.timestamp_ns);

        // img/meta are reused across calls (one persistent instance per
        // camera, passed in by the caller) so repeated publishes don't pay
        // for a fresh ~pixelBytes heap allocation every step -- data.assign()
        // reuses the vector's existing capacity once it has grown to size.
        img.header.stamp    = stamp;
        img.header.frame_id = frame_id;
        img.height          = static_cast<uint32_t>(height_);
        img.width           = static_cast<uint32_t>(width_);
        img.encoding        = image_encoding_;
        img.is_bigendian    = 0;
        img.step            = static_cast<uint32_t>(width_);
        img.data.assign(f.pixels, f.pixels + f.size);

        raw->publish(img);
        // f.ae_zone indexes directly into zone_pubs -- one image_raw topic
        // per AE zone (basler/camera_X/zone_{i}/image_raw).
        if (f.ae_zone >= 0 && static_cast<size_t>(f.ae_zone) < zone_pubs.size())
            zone_pubs[static_cast<size_t>(f.ae_zone)]->publish(img);

        meta.header.stamp    = stamp;
        meta.header.frame_id = frame_id;
        meta.exposure_us     = f.exposure_used_us;
        meta.gain_db         = f.gain_db;
        meta.brightness      = f.brightness;
        meta.ae_zone         = static_cast<uint8_t>(f.ae_zone);
        meta.sequence_index  = f.sequence_index;
        meta.converged       = f.converged;
        meta.track_unreachable = f.track_unreachable;
        // meta.thresholds stays empty (field kept for recorded bags).
        meta_pub->publish(meta);

        RCLCPP_DEBUG(get_logger(),
                     "[%s zone %d/%d] exp=%.1fµs gain=%.2fdB bright=%.1f converged=%s",
                     frame_id.c_str(), f.ae_zone, cfg_.aeZoneCount,
                     f.exposure_used_us, f.gain_db, f.brightness,
                     f.converged ? "yes" : "no");

        if (f.track_unreachable) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "[%s zone %d/%d] target brightness unreachable: exposure/gain "
                                  "pinned at their bounds (see min/max_exposure_us, min/max_gain_db)",
                                  frame_id.c_str(), f.ae_zone, cfg_.aeZoneCount);
        }
    }

    // ── Members ─────────────────────────────────────────────────────────────

    DriverConfig cfg_;
    bool         mono_ = false;  ///< camera_1 only (camera_2_serial empty after auto-detection).
    std::string  camera1_frame_id_;
    std::string  camera2_frame_id_;
    int          width_;
    int          height_;
    std::string  image_encoding_;

    std::unique_ptr<BracketingDriver> driver_;
    std::atomic<bool>                 running_{true};
    std::atomic<bool>                 failed_{false};
    std::thread                       capture_thread_;

    ImagePub camera1_raw_;
    MetaPub  camera1_meta_;
    std::vector<ImagePub> camera1_zone_pubs_;
    ImagePub camera2_raw_;
    MetaPub  camera2_meta_;
    std::vector<ImagePub> camera2_zone_pubs_;

    // Reused across publishFrame() calls so the ~pixelBytes data vector's
    // heap allocation happens once (on first grow) instead of every step.
    Image camera1_img_, camera2_img_;
    Meta  camera1_meta_msg_, camera2_meta_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        auto node = std::make_shared<BaslerCameraNode>();
        rclcpp::spin(node);
        rc = node->failed() ? 1 : 0;
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("basler_camera"), "Fatal: %s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
