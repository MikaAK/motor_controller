
# Bare-metal ARM GCC toolchain (macOS-safe)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_ROOT "/Users/mika/Documents/MotorController/arm/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi")
set(TOOLCHAIN_BIN  "${TOOLCHAIN_ROOT}/bin")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_BIN}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-gcc")

set(CMAKE_AR      "${TOOLCHAIN_BIN}/arm-none-eabi-ar")
set(CMAKE_OBJCOPY "${TOOLCHAIN_BIN}/arm-none-eabi-objcopy")
set(CMAKE_SIZE    "${TOOLCHAIN_BIN}/arm-none-eabi-size")

# IMPORTANT: prevents CMake from trying to run produced binaries during configure
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# IMPORTANT: stop CMake from “finding” macOS headers/libs/tools
set(CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_ROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
