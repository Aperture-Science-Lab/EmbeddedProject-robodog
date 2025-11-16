# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(spotmicro-rp2040 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(spotmicro-rp2040 
    spotmicro-rp2040.cpp
    h_bridge_l298n.cpp
)

pico_set_program_name(spotmicro-rp2040 "spotmicro-rp2040")
pico_set_program_version(spotmicro-rp2040 "0.1")

# Generate PIO header
pico_generate_pio_header(spotmicro-rp2040 ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(spotmicro-rp2040 0)
pico_enable_stdio_usb(spotmicro-rp2040 1)

# Add the standard library to the build
target_link_libraries(spotmicro-rp2040
        pico_stdlib
        hardware_gpio)

# Add the standard include files to the build
target_include_directories(spotmicro-rp2040 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(spotmicro-rp2040 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_timer
        hardware_clocks
        pico_cyw43_arch_none
        )

pico_add_extra_outputs(spotmicro-rp2040)

# ============================================================================
# Servo Driver Test Executable (PCA9685)
# ============================================================================

add_executable(test_servo
    test_servo.cpp
    pca9685.cpp
)

pico_set_program_name(test_servo "test_servo")
pico_set_program_version(test_servo "0.1")

pico_enable_stdio_uart(test_servo 0)
pico_enable_stdio_usb(test_servo 1)

target_include_directories(test_servo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(test_servo
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_add_extra_outputs(test_servo)

# ============================================================================
# Servo Debug/Diagnostic Tool
# ============================================================================

add_executable(test_servo_debug
    test_servo_debug.cpp
    pca9685.cpp
)

pico_set_program_name(test_servo_debug "test_servo_debug")
pico_set_program_version(test_servo_debug "0.1")

pico_enable_stdio_uart(test_servo_debug 0)
pico_enable_stdio_usb(test_servo_debug 1)

target_include_directories(test_servo_debug PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(test_servo_debug
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_add_extra_outputs(test_servo_debug)

# ============================================================================
# 12 Servo Test for SpotMicro
# ============================================================================

add_executable(test_12_servos
    test_12_servos.cpp
    pca9685.cpp
)

pico_set_program_name(test_12_servos "test_12_servos")
pico_set_program_version(test_12_servos "0.1")

pico_enable_stdio_uart(test_12_servos 0)
pico_enable_stdio_usb(test_12_servos 1)

target_include_directories(test_12_servos PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(test_12_servos
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_add_extra_outputs(test_12_servos)

# ============================================================================
# LCD 16x2 I2C Display Test
# ============================================================================

add_executable(test_lcd
    test_lcd.cpp
    lcd_16x2.cpp
)

pico_set_program_name(test_lcd "test_lcd")
pico_set_program_version(test_lcd "0.1")

pico_enable_stdio_uart(test_lcd 0)
pico_enable_stdio_usb(test_lcd 1)

target_include_directories(test_lcd PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(test_lcd
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_add_extra_outputs(test_lcd)

# ============================================================================
# HC-SR04 Ultrasonic Sensor Test
# ============================================================================

add_executable(test_hc_sr04
    test_hc_sr04.cpp
    hc_sr04.cpp
)

pico_set_program_name(test_hc_sr04 "test_hc_sr04")
pico_set_program_version(test_hc_sr04 "0.1")

pico_enable_stdio_uart(test_hc_sr04 0)
pico_enable_stdio_usb(test_hc_sr04 1)

target_include_directories(test_hc_sr04 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(test_hc_sr04
    pico_stdlib
    hardware_gpio
    hardware_timer
    m
)

pico_add_extra_outputs(test_hc_sr04)

# ============================================================================
# HC-SR04 GPIO Level Debugger
# ============================================================================

add_executable(test_hc_sr04_debug
    test_hc_sr04_debug.cpp
)

pico_set_program_name(test_hc_sr04_debug "test_hc_sr04_debug")
pico_set_program_version(test_hc_sr04_debug "0.1")

pico_enable_stdio_uart(test_hc_sr04_debug 0)
pico_enable_stdio_usb(test_hc_sr04_debug 1)

target_include_directories(test_hc_sr04_debug PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(test_hc_sr04_debug
    pico_stdlib
    hardware_gpio
)

pico_add_extra_outputs(test_hc_sr04_debug)

