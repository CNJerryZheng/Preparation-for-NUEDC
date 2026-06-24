# CMake最低版本
cmake_minimum_required(VERSION 3.20)

# 项目名称
set(PROJECT_NAME "empty")

# 链接脚本路径
set(LINKER_SCRIPT "../mspm0g3507.lds")

# 构建类型
set(CMAKE_BUILD_TYPE Debug)

# 添加可执行目标
add_executable(${PROJECT_NAME})

# 收集源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
	${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/startup_mspm0g350x_gcc.c
	${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/ti_msp_dl_config.c
	${CMAKE_CURRENT_SOURCE_DIR}/BSP/Src/board.c
	${CMAKE_CURRENT_SOURCE_DIR}/BSP/Src/oled.c
)

# 包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
	${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
)

# 驱动源文件
set(TI_Drivers_Src
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0l122x_l222x.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0l11xx_l13xx.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0g1x0x_g3x0x.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0c110x.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/m0p/dl_interrupt.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_adc12.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_vref.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_uart.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_trng.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_timer.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_spi.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_rtc_common.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_opa.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_mcan.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_mathacl.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_lfss.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_lcd.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_keystorectl.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_i2c.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_flashctl.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_dac12.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_crcp.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_crc.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_common.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_dma.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_aesadv.c
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/ti/driverlib/dl_aes.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${TI_Drivers_Src})

# 定义宏
set(PROJECT_DEFINES
	__MSPM0G3507__
)

# 编译器和工具链配置
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}ld)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# 项目定义
project(${PROJECT_NAME} C ASM)

# 驱动包含路径
set(DRIVER_INCLUDES
	${CMAKE_CURRENT_SOURCE_DIR}/Driver
	${CMAKE_CURRENT_SOURCE_DIR}/Driver/CMSIS/Core/Include
)

# 编译器宏定义和包含路径
add_compile_definitions(${PROJECT_DEFINES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${DRIVER_INCLUDES})

# 设置文件后缀
set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_STATIC_LIBRARY_SUFFIX ".a")

# C 编译器选项
if(${CMAKE_BUILD_TYPE} STREQUAL Debug)
	set(CMAKE_C_FLAGS "-mcpu=cortex-m0plus -std=gnu11 -mfloat-abi=soft --specs=nano.specs -mthumb -Wall -Og -g3 -ggdb -ffunction-sections -fdata-sections -fstack-usage -gstrict-dwarf")
elseif(${CMAKE_BUILD_TYPE} STREQUAL Release)
	set(CMAKE_C_FLAGS "-mcpu=cortex-m0plus -std=gnu11 -mfloat-abi=soft --specs=nano.specs -mthumb -Wall -Ofast -ffunction-sections -fdata-sections -fstack-usage -gstrict-dwarf")
endif()

# 汇编选项
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

# 链接器选项
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -T${LINKER_SCRIPT} --specs=nosys.specs -mthumb -static -lgcc -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -Wl,--start-group -lc -lm -Wl,--end-group -Wl,--print-memory-usage")

# 构建后命令：输出大小、HEX 和 BIN 文件
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)
