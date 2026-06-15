cmake_minimum_required(VERSION 2.8.0)

project(cvi_sample_detector C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT DEFINED TPU_SDK_PATH)
  message(FATAL_ERROR "Please set TPU_SDK_PATH to point to the TPU_SDK installation")
endif()
include_directories(${TPU_SDK_PATH}/include)
link_directories(${TPU_SDK_PATH}/lib)

if(NOT DEFINED OPENCV_PATH)
  message(FATAL_ERROR "Please set OPENCV_PATH to point to the opencvn installation")
endif()
include_directories(${OPENCV_PATH}/include)
link_directories(${OPENCV_PATH}/lib)

# 打印log等级

message(STATUS "Log level set to: ${LOG_LEVEL}")
if(NOT DEFINED LOG_LEVEL)
  set(LOG_LEVEL "INFO")
endif()

if(LOG_LEVEL STREQUAL "DEBUG")
  add_definitions(-DCURRENT_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "INFO")
  add_definitions(-DCURRENT_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "WARN")
  add_definitions(-DCURRENT_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "ERROR")
  add_definitions(-DCURRENT_LOG_LEVEL=3)
elseif(LOG_LEVEL STREQUAL "release")
  add_definitions(-DCURRENT_LOG_LEVEL=2)  # Release uses WARN level (only FPS and errors)
else()
  add_definitions(-DCURRENT_LOG_LEVEL=1) # Default to INFO
endif()

# Add sensor include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/include/isp/sg200x)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/include/common)

# Add parent directory include path for test files in tests/
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add sensor library path
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/lib)

set(CVI_LIBS ${CVI_LIBS} cviruntime cvikernel)
if(NOT CMAKE_CROSSCOMPILING)
  set(CVI_LIBS ${CVI_LIBS} cvicmodel)
endif()

set(OPENCV_LIBS ${OPENCV_LIBS} opencv_core opencv_imgcodecs opencv_imgproc)
if(NOT CMAKE_CROSSCOMPILING)
  set(OPENCV_LIBS ${OPENCV_LIBS} opencv_highgui)
endif()

set(EXTRA_LIBS ${EXTRA_LIBS} dl stdc++ pthread z atomic)

# Camera selection: USE_ESP32_CAMERA=ON uses ESP32-CAM via ioctl,
#                   USE_UVC_CAMERA=ON uses a USB UVC camera via V4L2 (/dev/videoN),
#                   both OFF uses VI+VPSS (MIPI sensor).
option(USE_ESP32_CAMERA "Use ESP32-CAM via /dev/cvi-camera ioctl instead of VI" OFF)
option(USE_UVC_CAMERA "Use a USB UVC camera via V4L2 instead of VI" OFF)
if(USE_ESP32_CAMERA)
  add_definitions(-DUSE_ESP32_CAMERA=1)
  message(STATUS "Camera: ESP32-CAM (ioctl mode)")
elseif(USE_UVC_CAMERA)
  add_definitions(-DUSE_UVC_CAMERA=1)
  message(STATUS "Camera: USB UVC (V4L2 mode)")
else()
  message(STATUS "Camera: VI + VPSS (hardware mode)")
endif()

# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commit 755a885
# 添加机械臂支持
# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commits 64f8dab, 7b7011d
# 添加通用检测模块支持

if(USE_ESP32_CAMERA)
  add_executable(tennis tennis.cpp motor.cpp pwm_motor_driver.cpp uart_motor_driver.cpp arm.cpp bucket_detect.cpp esp32_capture.cpp detect.cpp)
  target_link_libraries(tennis
      ${CVI_LIBS}
      ${OPENCV_LIBS}
      ${EXTRA_LIBS})
elseif(USE_UVC_CAMERA)
  add_executable(tennis tennis.cpp motor.cpp pwm_motor_driver.cpp uart_motor_driver.cpp arm.cpp bucket_detect.cpp uvc_capture.cpp detect.cpp)
  target_link_libraries(tennis
      ${CVI_LIBS}
      ${OPENCV_LIBS}
      ${EXTRA_LIBS})
else()
  add_executable(tennis tennis.cpp motor.cpp pwm_motor_driver.cpp uart_motor_driver.cpp arm.cpp bucket_detect.cpp vi_capture.cpp detect.cpp)
  target_link_libraries(tennis
      ${CVI_LIBS}
      ${OPENCV_LIBS}
      -Wl,--start-group
      sample isp isp_algo ae awb af
      vpu cvi_ispd2 cvi_bin_isp cvi_bin
      sns_full sys ini json-c
      -Wl,--end-group
      ${EXTRA_LIBS})
endif()
install(TARGETS tennis DESTINATION samples_extra/bin)

# =====================================================
# Test utilities (in tests/ directory)
# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commits 755a885, 64f8dab
# =====================================================

# Arm test utility (no SDK dependencies needed)
add_executable(test_arm tests/test_arm.cpp arm.cpp)

# Motor test utility (no SDK dependencies needed)
add_executable(test_motor tests/test_motor.cpp motor.cpp pwm_motor_driver.cpp uart_motor_driver.cpp)

# UART probe utility: send INIT frame and check for a valid ACK, to find which
# /dev/ttySx is the ESP32-C3 motor controller (no SDK dependencies needed)
add_executable(test_uart_probe tests/test_uart_probe.cpp)

# ESP32 camera standalone test (no cvitek SDK / no TPU dependencies)
# Verifies /dev/cvi-camera0 ioctl path through cvi_camera.rs kernel driver.
add_executable(test_esp32_capture tests/test_esp32_capture.cpp)

# UVC camera standalone test (V4L2, no cvitek SDK / no TPU dependencies)
# Opens /dev/videoN, grabs frames, dumps last frame to verify UVC capture.
add_executable(test_uvc_capture tests/test_uvc_capture.cpp uvc_capture.cpp)

# Single-frame capture + detection utility
# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commits 64f8dab, 7b7011d
# 使用通用检测模块
add_executable(test_capture_detect tests/test_capture_detect.cpp vi_capture.cpp detect.cpp)
target_link_libraries(test_capture_detect
    ${CVI_LIBS}
    ${OPENCV_LIBS}
    -Wl,--start-group
    sample isp isp_algo ae awb af
    vpu cvi_ispd2 cvi_bin_isp cvi_bin
    sns_full sys ini json-c
    -Wl,--end-group
    ${EXTRA_LIBS})

# Static image detection utility
# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commit 64f8dab
# 使用通用检测模块
add_executable(test_img_detect tests/test_img_detect.cpp detect.cpp)
target_link_libraries(test_img_detect
    ${CVI_LIBS}
    ${OPENCV_LIBS}
    ${EXTRA_LIBS})

# YOLO detection test utility (UVC/VI camera + letterbox + YOLO inference)
# 用法: test_yolo <model.cvimodel> [uvc_index=0] [output.jpg]
if(USE_ESP32_CAMERA)
  add_executable(test_yolo tests/test_yolo.cpp esp32_capture.cpp detect.cpp)
elseif(USE_UVC_CAMERA)
  add_executable(test_yolo tests/test_yolo.cpp uvc_capture.cpp detect.cpp)
else()
  add_executable(test_yolo tests/test_yolo.cpp vi_capture.cpp detect.cpp)
endif()
target_link_libraries(test_yolo
    ${CVI_LIBS}
    ${OPENCV_LIBS}
    ${EXTRA_LIBS})