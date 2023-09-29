cmake_minimum_required(VERSION 3.27)

# M46x M467 M467SJHAN
project(lightkraken2 C CXX ASM)

set(BASE_ADDRESS 0x00000000)

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

set(THREADX_ARCH cortex_m4)
set(THREADX_TOOLCHAIN gnu)

add_compile_definitions(
    TX_MAX_PRIORITIES=32
    TX_DISABLE_PREEMPTION_THRESHOLD
    TX_DISABLE_REDUNDANT_CLEARING
    TX_DISABLE_NOTIFY_CALLBACKS
    TX_NOT_INTERRUPTABLE
    TX_TIMER_PROCESS_IN_ISR
    TX_REACTIVATE_INLINE
    TX_DISABLE_STACK_FILLING
    TX_INLINE_THREAD_RESUME_SUSPEND
    __HXT=12000000
    __FPU_PRESENT=1U
    NDEBUG 
    ARM_MATH_CM4)

set(BSP_DIR ${PROJECT_SOURCE_DIR}/m460bsp)
set(THREADX_DIR ${PROJECT_SOURCE_DIR}/threadx)
set(FILEX_DIR ${PROJECT_SOURCE_DIR}/filex)
set(NETXDUO_DIR ${PROJECT_SOURCE_DIR}/netxduo)

set(LD_SCRIPT ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/GCC/gcc_arm.ld)

set(CC_FLAGS
    -flto=auto
    -mcpu=cortex-m4
    -Wformat=2
    -Wformat-overflow
    -Wformat-truncation
    -Wnull-dereference
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -Wpedantic 
    -Wfloat-conversion 
    -Wdouble-promotion
    -Wno-unused-parameter
    -Wno-strict-aliasing 
    -Wno-format
    -Wstack-usage=8192
    -Wuninitialized
    -funsigned-char
    -fshort-enums
    -fshort-wchar
    -fno-common
    -fstack-usage
    -ffast-math
    -ffat-lto-objects
    -fallow-store-data-races)

set(CXX_FLAGS 
    -Weffc++
    -Wsign-conversion
    -Wshadow 
    -Wnon-virtual-dtor
    -Wunused
    -Woverloaded-virtual
    -Wmisleading-indentation
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wstrict-aliasing 
    -Wcast-align
    -Wconversion
    -Wno-volatile
    -std=c++20
    -fno-rtti 
    -fno-exceptions)

set(LD_FLAGS
    -T${LD_SCRIPT}
    -u _printf_float
    -flto=auto
    -specs=nano.specs
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -lm
    -lnosys
    -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections)

set(DEBUG_FLAGS
    -Og -g)

set(RELEASE_FLAGS   
    -Os)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(DUMP_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.S)

add_executable(${PROJECT_NAME}.elf 
    ${PROJECT_SOURCE_DIR}/main.c
    ${PROJECT_SOURCE_DIR}/stubs.c
    ${PROJECT_SOURCE_DIR}/tx_initialize_low_level.c
    ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/GCC/startup_M460.S
    ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/system_m460.c
)

include_directories(${BSP_DIR}/Library/Device/Nuvoton/m460/Include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)

target_include_directories(${PROJECT_NAME}.elf PRIVATE ${CMAKE_BINARY_DIR})

add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
target_link_libraries(${PROJECT_NAME}.elf standard_driver)

add_subdirectory(${THREADX_DIR})
target_link_libraries(${PROJECT_NAME}.elf threadx)

add_subdirectory(${FILEX_DIR})
target_link_libraries(${PROJECT_NAME}.elf filex)

add_subdirectory(${NETXDUO_DIR})
target_link_libraries(${PROJECT_NAME}.elf netxduo)

# Generate version.h
find_package(Git)

execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_SHORT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${GIT_EXECUTABLE} rev-list HEAD --count WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_REV_COUNT OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${GIT_EXECUTABLE} show -s --format=%ad HEAD --date=iso-strict WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_COMMIT_DATE OUTPUT_STRIP_TRAILING_WHITESPACE)

configure_file("${PROJECT_SOURCE_DIR}/version.h.in" "${CMAKE_BINARY_DIR}/version.h" @ONLY)

target_compile_definitions(standard_driver PRIVATE ${DEFINITIONS})
target_compile_options(standard_driver PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_FLAGS}>")
target_compile_options(standard_driver PRIVATE "$<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>")
target_compile_options(standard_driver PRIVATE "$<$<CONFIG:RELEASE>:${RELEASE_FLAGS}>")

target_compile_definitions(threadx PRIVATE ${DEFINITIONS})
target_compile_options(threadx PRIVATE "$<$<COMPILE_LANGUAGE:ASM>:${CC_FLAGS}>")
target_compile_options(threadx PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_FLAGS}>")
target_compile_options(threadx PRIVATE "$<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>")
target_compile_options(threadx PRIVATE "$<$<CONFIG:RELEASE>:${RELEASE_FLAGS}>")

target_compile_definitions(filex PRIVATE ${DEFINITIONS})
target_compile_options(filex PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_FLAGS}>")
target_compile_options(filex PRIVATE "$<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>")
target_compile_options(filex PRIVATE "$<$<CONFIG:RELEASE>:${RELEASE_FLAGS}>")

target_compile_definitions(netxduo PRIVATE ${DEFINITIONS})
target_compile_options(netxduo PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_FLAGS}>")
target_compile_options(netxduo PRIVATE "$<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>")
target_compile_options(netxduo PRIVATE "$<$<CONFIG:RELEASE>:${RELEASE_FLAGS}>")

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${DEFINITIONS})
target_link_options(${PROJECT_NAME}.elf PRIVATE ${LD_FLAGS})
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_FLAGS}>")
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CXX_FLAGS}>")
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>")
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<CONFIG:RELEASE>:${RELEASE_FLAGS}>")

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -D $<TARGET_FILE:${PROJECT_NAME}.elf> > ${DUMP_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signature $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")

message(${PROJECT_SOURCE_DIR})

set(PROGRAM_CMD "./openocd -f ./nulink.cfg -f ./numicroM4.cfg -c \"program ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin ${BASE_ADDRESS} verify reset exit\"")
install(CODE "execute_process(COMMAND ${PROGRAM_CMD} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/openocd)")
