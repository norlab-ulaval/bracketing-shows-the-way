cmake_minimum_required(VERSION 3.16)
project(basler_bracketing_driver CXX)

# Static library consumed by ../driver_ros via add_subdirectory(). It can
# also be configured on its own as a compile check:
#   cmake -S driver -B build && cmake --build build

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ── Basler pylon SDK (Linux), located through pylon-config ─────────────────
find_program(PYLON_CONFIG pylon-config REQUIRED
    HINTS $ENV{PYLON_ROOT}/bin /opt/pylon/bin)
execute_process(COMMAND ${PYLON_CONFIG} --cflags-only-I
    OUTPUT_VARIABLE _PYLON_INC OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${PYLON_CONFIG} --libs
    OUTPUT_VARIABLE _PYLON_LIBS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${PYLON_CONFIG} --libs-rpath
    OUTPUT_VARIABLE _PYLON_RPATH OUTPUT_STRIP_TRAILING_WHITESPACE)
# --cflags-only-I prints "-I<dir>"; target_include_directories wants <dir>.
string(REPLACE "-I" "" _PYLON_INC "${_PYLON_INC}")
separate_arguments(PYLON_INCLUDE_DIRS UNIX_COMMAND "${_PYLON_INC}")
separate_arguments(PYLON_LIBS         UNIX_COMMAND "${_PYLON_LIBS}")
separate_arguments(PYLON_RPATH_FLAGS  UNIX_COMMAND "${_PYLON_RPATH}")

add_library(basler_bracketing_driver STATIC
    src/BracketingDriver.cpp
    src/CameraGate.cpp
    src/Config.cpp
    src/ZoneManager.cpp
    src/ExposureTrack.cpp
)
target_include_directories(basler_bracketing_driver
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC  ${PYLON_INCLUDE_DIRS})
find_package(Threads REQUIRED)
target_link_libraries(basler_bracketing_driver PUBLIC ${PYLON_LIBS} Threads::Threads)
# Bake the pylon library directory into the RPATH of executables linking this.
target_link_options(basler_bracketing_driver PUBLIC ${PYLON_RPATH_FLAGS})

message(STATUS "pylon includes: ${PYLON_INCLUDE_DIRS}")
message(STATUS "pylon libs    : ${PYLON_LIBS}")
