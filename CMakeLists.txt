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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")


# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ATIVIDADE_IMU C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

set(FREERTOS_KERNEL_PATH "C:/Users/T-GAMER/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Add executable. Default name is the project name, version 0.1
include_directories( ${CMAKE_SOURCE_DIR}/lib )
add_subdirectory(lib/FatFs_SPI) 

add_executable(ATIVIDADE_IMU ATIVIDADE_IMU.c 
        hw_config.c
        ssd1306.c
        pio_matrix.pio)

pico_set_program_name(ATIVIDADE_IMU "ATIVIDADE_IMU")
pico_set_program_version(ATIVIDADE_IMU "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ATIVIDADE_IMU 1)
pico_enable_stdio_usb(ATIVIDADE_IMU 1)

pico_generate_pio_header(ATIVIDADE_IMU ${CMAKE_CURRENT_LIST_DIR}/pio_matrix.pio)

# Add the standard library to the build
target_link_libraries(ATIVIDADE_IMU
        pico_stdlib
        FatFs_SPI
        hardware_clocks
        hardware_adc
        hardware_i2c
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        hardware_clocks
        hardware_pio
        hardware_pwm)

# Add the standard include files to the build
target_include_directories(ATIVIDADE_IMU PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(ATIVIDADE_IMU 
        
        )

pico_add_extra_outputs(ATIVIDADE_IMU)

