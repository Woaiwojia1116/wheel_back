cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FOCtest)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/IIC.c

    ${CMAKE_CURRENT_SOURCE_DIR}/User_driver/delay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_driver/filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_driver/foc_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_driver/MagneticSensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_driver/mpu6050.c

    ${CMAKE_CURRENT_SOURCE_DIR}/User_app/FOC.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_app/lqr_ctrl.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/User_app
    ${CMAKE_CURRENT_SOURCE_DIR}/User_driver
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 启用 newlib-nano 的浮点 printf（源码中零电角调试打印使用 %.4f）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-u,_printf_float)

# 临时固定速度测试模式（默认关闭）：两轮固定 1.0 rad/s，用于观察速度环
option(FOC_SPEED_TEST "Enable fixed-speed motor test mode" OFF)
if(FOC_SPEED_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_SPEED_TEST=1)
endif()
