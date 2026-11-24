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
set(CMAKE_PROJECT_NAME Transport_car)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/Moto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/Gimbal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/Chassis.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/chassis_nav.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/chassis_runtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Source/chassis_bus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Source/YB_MRA02_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/Vision.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/Navigation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/IMU.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Source/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Source/bsp_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Src/pid.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/Inc
    # Add user defined include paths
)

# Add project symbols (macros)
option(CHASSIS_OFFLINE_30CM "Run the distance test once after boot" OFF)
option(CHASSIS_OFFLINE_TURN90 "Run a counterclockwise 90-degree turn once after boot" OFF)
set(CHASSIS_TEST_DISTANCE_MM 300.0 CACHE STRING "Straight-line test distance in mm")
set(CHASSIS_TEST_TIMEOUT_MS 8000 CACHE STRING "Straight-line test timeout in ms")
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<BOOL:${CHASSIS_OFFLINE_30CM}>:CHASSIS_OFFLINE_30CM>
    $<$<BOOL:${CHASSIS_OFFLINE_TURN90}>:CHASSIS_OFFLINE_TURN90>
    CHASSIS_TEST_DISTANCE_MM=${CHASSIS_TEST_DISTANCE_MM}f
    CHASSIS_TEST_TIMEOUT_MS=${CHASSIS_TEST_TIMEOUT_MS}U
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
