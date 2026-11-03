# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pico_pilot C CXX ASM)

pico_sdk_init()

add_executable(pico_pilot
  bno055.c
  drive_control.c
  encoders.c
  microros_time.c
  pico_pilot.c
  picow_udp_transport.c
  tb6612.c
  ultrasonic.c
)

# Added: import the Cortex-M33 micro-ROS archive and headers.
add_library(micro_ros STATIC IMPORTED)
set_target_properties(micro_ros PROPERTIES
  IMPORTED_LOCATION
    "${CMAKE_CURRENT_LIST_DIR}/libmicroros/libmicroros.a"
  INTERFACE_INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_LIST_DIR}/libmicroros/include"
)

pico_set_program_name(pico_pilot "pico_pilot")
pico_set_program_version(pico_pilot "0.1")

pico_enable_stdio_uart(pico_pilot 0)
pico_enable_stdio_usb(pico_pilot 1)

target_include_directories(pico_pilot PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Updated: one link block contains the Pico SDK, Wi-Fi, and micro-ROS.
target_link_libraries(pico_pilot
  pico_stdlib
  pico_cyw43_arch_lwip_threadsafe_background
  hardware_gpio
  hardware_i2c
  hardware_pwm
  micro_ros
)

pico_add_extra_outputs(pico_pilot)