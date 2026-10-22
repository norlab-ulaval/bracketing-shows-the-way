# Per-frame metadata published alongside each raw image (basler/camera_X/image_meta).
# The header stamp and frame_id match the corresponding image_raw message. The stamp
# is host time taken right after both cameras' grabs returned (not exposure start).
# The field layout matches the recorded datasets; do not reorder or remove fields.

std_msgs/Header header

float64 exposure_us     # Exposure actually applied to this frame (µs), from camera chunk data.
float64 gain_db         # Gain programmed on this camera for this frame (dB). Not read back
                        # from the camera.
float64 brightness      # camera_1's AE measurement for this frame's zone (0-255, gray from
                        # 2x2 Bayer blocks): the measured percentile of the whole frame
                        # (zone 0: ae_bridge_low_percentile, zone 2: ae_bridge_high_percentile,
                        # zone 1: median, informational). camera_2 frames carry camera_1's value.
uint8   ae_zone         # Zone the frame was attributed to: 0 = low (darkest bracket), 1 = mid,
                        # 2 = high.
uint8   sequence_index  # Position in the ping-pong cycle 0,1,2,1: 0..3.
bool    converged       # This zone's controller has converged (last 3 readings within
                        # epsilon). For mid: low and high both converged.
bool    track_unreachable # Target not reachable with exposure and gain pinned at their bounds
                        # (min/max_exposure_us, min/max_gain_db).
uint8[] thresholds      # Always empty (B-Adapt has no intensity bands); kept so that
                        # recorded bags remain readable.

# Unused by basler_camera_node (always 0 / false); kept for compatibility with
# recorded bags.
uint16  bracket_index
float64 requested_exposure_us
bool    is_dark_frame
