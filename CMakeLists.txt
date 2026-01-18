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

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ROBOT_CPP)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# We are not using C++ modules; avoid compiler/module scanning flags to keep
# tooling (clangd) simple.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF CACHE BOOL "" FORCE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY CXX_SCAN_FOR_MODULES OFF)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    
    Core/Src/app_i2c.c
    Core/Src/app_params.c
    Core/Src/app_spi.c
    Core/Src/app_rtos.c
    RobotApp/Bridge/app_entry.cpp
    RobotApp/Estimation/ins_estimator.cpp
    RobotApp/Algo/user_controller.cpp
    RobotApp/Platform/heap_guard.cpp
    RobotApp/Tasks/control_loop.cpp
    RobotApp/Tasks/sensor_task.cpp
    RobotApp/Tasks/logic_task.cpp
    RobotApp/Tasks/sbus_task.cpp
    RobotApp/Inputs/sbus.cpp
    RobotApp/Drivers/bmi088.cpp
    RobotApp/Drivers/bmi088_dma_transport.c
    RobotApp/Drivers/ist8310.cpp
    RobotApp/Drivers/motor_can_tx_callback.cpp
    imu/bmi08a.c
    imu/bmi08g.c
    imu/drv_bmi088.c
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/RobotApp/Bridge
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
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

# Hard guard: trap if any heap operation occurs after scheduler start.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--wrap=malloc
    -Wl,--wrap=free
    -Wl,--wrap=calloc
    -Wl,--wrap=realloc
)
