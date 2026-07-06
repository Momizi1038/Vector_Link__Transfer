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
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Vector_Link_Transfer_v1 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(Vector_Link_Transfer_v1
    server.cpp
    bluetooth_driver_server.cpp
    hid_app.c
)

target_sources(Vector_Link_Transfer_v1 PRIVATE
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
)

add_compile_definitions(
    CFG_TUSB_MCU=OPT_MCU_RP2040
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_HOST
    CFG_TUH_ENABLED=1
)

add_subdirectory(lib/Pico-PIO-USB)

# Add executable. Default name is the project name, version 0.1
pico_set_program_name(Vector_Link_Transfer_v1 "Vector_Link_Transfer")
pico_set_program_version(Vector_Link_Transfer_v1 "0.1")

# Standalone example that reads from the on board temperature sensor and sends notifications via BLE
# Flashes slowly each second to show it's running
target_link_libraries(Vector_Link_Transfer_v1
    pico_stdlib
    pico_btstack_classic  
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    pico_rand
    pio_usb_host
    hardware_uart
    tinyusb_board
    tinyusb_host
    )
target_include_directories(Vector_Link_Transfer_v1 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # For our common btstack config
    lib/Pico-PIO-USB
    )
# pico_btstack_make_gatt_header(Vector_Link_Transfer_v1 PRIVATE "${CMAKE_CURRENT_LIST_DIR}/temp_sensor.gatt")

pico_enable_stdio_usb(Vector_Link_Transfer_v1 0)
pico_enable_stdio_uart(Vector_Link_Transfer_v1 1)

pico_add_extra_outputs(Vector_Link_Transfer_v1)
