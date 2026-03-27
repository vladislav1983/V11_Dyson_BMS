cmake_minimum_required(VERSION 3.20)

project(samd20_firmware C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type: Debug or Release" FORCE)
endif()

# -----------------------------------------------------------------------------
# MCU / CPU
# -----------------------------------------------------------------------------
set(MCU_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
)

# -----------------------------------------------------------------------------
# Compiler flags
# -----------------------------------------------------------------------------
set(COMMON_C_FLAGS
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -pipe
    -fno-strict-aliasing
    -Wall
    -Wstrict-prototypes
    -Wmissing-prototypes
    -Werror-implicit-function-declaration
    -Wpointer-arith
    -std=gnu99
    -Wchar-subscripts
    -Wcomment
    -Wformat=2
    -Wimplicit-int
    -Wmain
    -Wparentheses
    -Wsequence-point
    -Wreturn-type
    -Wswitch
    -Wtrigraphs
    -Wunused
    -Wuninitialized
    -Wunknown-pragmas
    -Wfloat-equal
    -Wundef
    -Wshadow
    -Wbad-function-cast
    -Wwrite-strings
    -Wsign-compare
    -Waggregate-return
    -Wmissing-declarations
    -Wformat
    -Wmissing-format-attribute
    -Wno-deprecated-declarations
    -Wpacked
    -Wredundant-decls
    -Wnested-externs
    -Wlong-long
    -Wunreachable-code
    -Wcast-align
    --param max-inline-insns-single=500
    -Wno-overflow
)

add_compile_options(${COMMON_C_FLAGS})

# Debug:   -Og, debug symbols
# Release: -Os, no debug symbols
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Og -g3)
else()
    add_compile_options(-Os)
    add_compile_definitions(NDEBUG)
endif()

# -----------------------------------------------------------------------------
# Compile definitions
# -----------------------------------------------------------------------------
add_compile_definitions(
    __SAMD20E15__
    BOARD=USER_BOARD
    ARM_MATH_CM0PLUS=true
    I2C_MASTER_CALLBACK_MODE=false
    SYSTICK_MODE
    ADC_CALLBACK_MODE=false
    EXTINT_CALLBACK_MODE=true
    USART_CALLBACK_MODE=true
    TC_ASYNC=true
    WDT_CALLBACK_MODE=true
    RTC_COUNT_ASYNC=true
)

# -----------------------------------------------------------------------------
# Linker flags
# -----------------------------------------------------------------------------
set(LINKER_SCRIPT
    ${CMAKE_SOURCE_DIR}/src/ASF/sam0/utils/linker_scripts/samd20/gcc/samd20e15_flash.ld
)

add_link_options(
    ${MCU_FLAGS}
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    --specs=nosys.specs
)

# -----------------------------------------------------------------------------
# Include directories
# -----------------------------------------------------------------------------
include_directories(
    src
    src/config

    src/ASF/common/boards
    src/ASF/common/utils
    src/ASF/common/services/ioport
    src/ASF/common2/boards/user_board
    src/ASF/common2/services/delay
    src/ASF/common2/services/delay/sam0
    src/ASF/sam0/utils
    src/ASF/sam0/utils/header_files
    src/ASF/sam0/utils/preprocessor
    src/ASF/sam0/utils/cmsis/samd20/include
    src/ASF/sam0/utils/cmsis/samd20/source
    src/ASF/sam0/drivers/system
    src/ASF/sam0/drivers/system/clock
    src/ASF/sam0/drivers/system/clock/clock_samd20
    src/ASF/sam0/drivers/system/interrupt
    src/ASF/sam0/drivers/system/interrupt/system_interrupt_samd20
    src/ASF/sam0/drivers/system/pinmux
    src/ASF/sam0/drivers/system/power
    src/ASF/sam0/drivers/system/power/power_sam_d_r_h
    src/ASF/sam0/drivers/system/reset
    src/ASF/sam0/drivers/system/reset/reset_sam_d_r_h
    src/ASF/sam0/drivers/port
    src/ASF/sam0/drivers/sercom
    src/ASF/sam0/drivers/sercom/i2c
    src/ASF/sam0/drivers/sercom/i2c/i2c_samd20
    src/ASF/sam0/drivers/sercom/usart
    src/ASF/sam0/drivers/extint
    src/ASF/sam0/drivers/extint/extint_sam_d_r_h
    src/ASF/sam0/drivers/adc
    src/ASF/sam0/drivers/adc/adc_sam_d_r_h
    src/ASF/sam0/drivers/nvm
    src/ASF/sam0/drivers/rtc
    src/ASF/sam0/drivers/tc
    src/ASF/sam0/drivers/wdt
    src/ASF/sam0/services/eeprom/emulator/main_array
    src/ASF/thirdparty/CMSIS/Include
    src/ASF/thirdparty/CMSIS/Lib/GCC
)

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
file(GLOB_RECURSE SOURCES src/*.c)

# Define the ELF target
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# -----------------------------------------------------------------------------
# Size + HEX generation
# -----------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${PROJECT_NAME}.hex
)

# -----------------------------------------------------------------------------
# Link libraries
# -----------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}.elf m)

# -----------------------------------------------------------------------------
# OpenOCD flash target
# -----------------------------------------------------------------------------
set(OPENOCD_CFG ${CMAKE_SOURCE_DIR}/openocd_samd20.cfg)

add_custom_target(flash
    COMMAND openocd
        -f ${OPENOCD_CFG}
        -c "program $<TARGET_FILE:${PROJECT_NAME}.elf> verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Flashing ${PROJECT_NAME} via OpenOCD"
)

# Unlock (and fully erase) a Dyson pack with firmware protection enabled.
# The device will need to be programmed after this.
add_custom_target(unlock
    COMMAND openocd
        -f ${OPENOCD_CFG}
        -c "\"init; at91samd.cpu mwb 0x41002100 0x10; sleep 500; reset; halt; exit\""
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Unlocking firmware-protected Dyson pack via OpenOCD"
)
