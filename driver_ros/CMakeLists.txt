cmake_minimum_required(VERSION 3.19)
project(basler_camera C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/BaslerFrameMeta.msg"
    DEPENDENCIES std_msgs
)

# ── Camera driver library (../driver, plain CMake, needs the pylon SDK) ─────
# Resolve symlinks so this also works when driver_ros is symlinked into a
# colcon workspace.
file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." _repo_root)
set(BASLER_DRIVER_DIR "${_repo_root}/driver" CACHE PATH
    "Path to the basler bracketing driver library (contains CMakeLists.txt)")
add_subdirectory(${BASLER_DRIVER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/driver EXCLUDE_FROM_ALL)

add_executable(basler_camera_node src/basler_camera_node.cpp)
target_link_libraries(basler_camera_node basler_bracketing_driver)
ament_target_dependencies(basler_camera_node rclcpp rcl_interfaces sensor_msgs std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(basler_camera_node "${cpp_typesupport_target}")

install(TARGETS basler_camera_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
