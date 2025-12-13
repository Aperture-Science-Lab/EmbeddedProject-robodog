cmake_minimum_required(VERSION 3.13)

# =========================================================================
# Set board to Pico W
# =========================================================================
set(PICO_BOARD pico_w)

# Import Pico SDK
include(pico_sdk_import.cmake)

# --- FREERTOS CONFIGURATION (must be BEFORE project()) ---
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)
set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Import FreeRTOS (MUST be before project() for proper integration)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# --- PROJECT SETTINGS ---
project(SpotMicro C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize SDK
pico_sdk_init()

# --- SOURCE FILES ---
add_executable(SpotMicro
    main_controller.cpp
    sensor_hub.cpp
    drivers/pca9685.cpp
    drivers/lcd_16x2.cpp
    drivers/hc_sr04.cpp
    drivers/h_bridge_l298n.cpp
    middleware/kinematics/spot_kinematics_v2.cpp
)

# --- INCLUDE PATHS ---
target_include_directories(SpotMicro PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    ${CMAKE_CURRENT_LIST_DIR}/middleware/kinematics
)

# --- LIBRARIES TO LINK ---
target_link_libraries(SpotMicro 
    pico_stdlib
    pico_stdio_usb
    hardware_i2c
    hardware_uart
    hardware_pwm
    hardware_gpio
    hardware_flash
    tinyusb_device
    tinyusb_board
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# --- USB OUTPUT (CRITICAL - must be enabled for serial debugging) ---
pico_enable_stdio_usb(SpotMicro 1)
pico_enable_stdio_uart(SpotMicro 0)

pico_add_extra_outputs(SpotMicro)