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
set(PICO_BOARD pico_w)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SSI_rocketry C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
set (SOURCES
    src/main.c
    src/devices/imu.c
    src/devices/barometric_sensor/MS5611.c
    )

add_executable(SSI_rocketry ${SOURCES} )

add_subdirectory(libs/FatFs_SPI)


pico_set_program_name(SSI_rocketry "SSI_rocketry")
pico_set_program_version(SSI_rocketry "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(SSI_rocketry 0)
pico_enable_stdio_usb(SSI_rocketry 1)

# Add the standard library to the build
target_link_libraries(SSI_rocketry
        pico_stdlib
        hardware_i2c
        FatFs_SPI
        pico_cyw43_arch_none
        )

# Add the standard include files to the build
target_include_directories(SSI_rocketry PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

pico_add_extra_outputs(SSI_rocketry)

