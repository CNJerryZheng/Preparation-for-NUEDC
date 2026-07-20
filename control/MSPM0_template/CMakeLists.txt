cmake_minimum_required(VERSION 3.20)

if(CMAKE_GENERATOR MATCHES "Visual Studio")
    message(FATAL_ERROR
        "This is an ARM cross-compiled project. Select the arm-gcc-debug "
        "CMake preset (Ninja); the Visual Studio generator uses cl.exe.")
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_GCC arm-none-eabi-gcc REQUIRED)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size REQUIRED)

set(CMAKE_C_COMPILER "${ARM_GCC}")
set(CMAKE_ASM_COMPILER "${ARM_GCC}")

project(empty LANGUAGES C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(PROJECT_TARGET empty)
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/mspm0g3507.lds")

set(APP_SOURCES
    Core/Src/main.c
    Core/Src/startup_mspm0g350x_gcc.c
    Core/Src/ti_msp_dl_config.c
    BSP/Src/board.c
)

# DriverLib 中大多数接口是内联函数；芯片族相关实现只编译 MSPM0G。
set(DRIVER_SOURCES
    Driver/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0g1x0x_g3x0x.c
    Driver/ti/driverlib/m0p/dl_interrupt.c
    Driver/ti/driverlib/dl_adc12.c
    Driver/ti/driverlib/dl_vref.c
    Driver/ti/driverlib/dl_uart.c
    Driver/ti/driverlib/dl_trng.c
    Driver/ti/driverlib/dl_timer.c
    Driver/ti/driverlib/dl_spi.c
    Driver/ti/driverlib/dl_rtc_common.c
    Driver/ti/driverlib/dl_opa.c
    Driver/ti/driverlib/dl_mcan.c
    Driver/ti/driverlib/dl_mathacl.c
    Driver/ti/driverlib/dl_lfss.c
    Driver/ti/driverlib/dl_lcd.c
    Driver/ti/driverlib/dl_keystorectl.c
    Driver/ti/driverlib/dl_i2c.c
    Driver/ti/driverlib/dl_flashctl.c
    Driver/ti/driverlib/dl_dac12.c
    Driver/ti/driverlib/dl_crcp.c
    Driver/ti/driverlib/dl_crc.c
    Driver/ti/driverlib/dl_common.c
    Driver/ti/driverlib/dl_dma.c
    Driver/ti/driverlib/dl_aesadv.c
    Driver/ti/driverlib/dl_aes.c
)

add_executable(${PROJECT_TARGET} ${APP_SOURCES} ${DRIVER_SOURCES})
set_target_properties(${PROJECT_TARGET} PROPERTIES SUFFIX ".elf")

target_include_directories(${PROJECT_TARGET} PRIVATE
    Core/Inc
    BSP/Inc
)

target_include_directories(${PROJECT_TARGET} SYSTEM PRIVATE
    Driver
    Driver/CMSIS/Core/Include
)

target_compile_definitions(${PROJECT_TARGET} PRIVATE __MSPM0G3507__)

target_compile_options(${PROJECT_TARGET} PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    --specs=nano.specs
    -ffunction-sections
    -fdata-sections
    -fstack-usage
    -Wall
    -Wextra
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
    "$<$<CONFIG:Debug>:-Og;-g3;-ggdb>"
    $<$<CONFIG:Release>:-O2>
)

# DriverLib 按芯片裁剪后会留下少量未使用形参，不让它们淹没应用层告警。
set_source_files_properties(${DRIVER_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-parameter"
)

target_link_options(${PROJECT_TARGET} PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
    -static
    "-T${LINKER_SCRIPT}"
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_TARGET}.map"
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

target_link_libraries(${PROJECT_TARGET} PRIVATE
    -Wl,--start-group
    c
    m
    gcc
    -Wl,--end-group
)

add_custom_command(TARGET ${PROJECT_TARGET} POST_BUILD
    COMMAND "${ARM_SIZE}" "$<TARGET_FILE:${PROJECT_TARGET}>"
    COMMAND "${ARM_OBJCOPY}" -O ihex "$<TARGET_FILE:${PROJECT_TARGET}>" "${PROJECT_TARGET}.hex"
    COMMAND "${ARM_OBJCOPY}" -O binary "$<TARGET_FILE:${PROJECT_TARGET}>" "${PROJECT_TARGET}.bin"
    COMMENT "Generating size report, HEX and BIN files"
)
