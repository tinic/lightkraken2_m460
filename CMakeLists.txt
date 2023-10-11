cmake_minimum_required(VERSION 3.0...3.16)

# M46x M467 M467SJHAN
project(lightkraken2 C CXX ASM)

set(BASE_ADDRESS 0x00000000)

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

set(THREADX_ARCH cortex_m4)
set(THREADX_TOOLCHAIN gnu)

add_compile_definitions(
    TX_TIMER_TICKS_PER_SECOND=240
    TX_MAX_PRIORITIES=32
    TX_DISABLE_PREEMPTION_THRESHOLD
    TX_DISABLE_NOTIFY_CALLBACKS
    TX_DISABLE_REDUNDANT_CLEARING
    TX_NOT_INTERRUPTABLE
    TX_TIMER_PROCESS_IN_ISR
    TX_REACTIVATE_INLINE
    TX_INLINE_THREAD_RESUME_SUSPEND
    __HXT=12000000UL
    __FPU_PRESENT=1U
    ARM_MATH_CM4
    $<$<NOT:$<CONFIG:Release>>:TX_ENABLE_STACK_CHECKING>
    $<$<CONFIG:Release>:TX_DISABLE_STACK_FILLING>
    $<$<CONFIG:Release>:TX_DISABLE_ERROR_CHECKING>
    $<$<CONFIG:Release>:NDEBUG>)

set(BSP_DIR ${PROJECT_SOURCE_DIR}/m460bsp)
set(THREADX_DIR ${PROJECT_SOURCE_DIR}/threadx)
set(FILEX_DIR ${PROJECT_SOURCE_DIR}/filex)
set(NETXDUO_DIR ${PROJECT_SOURCE_DIR}/netxduo)
set(LWJSON_DIR ${PROJECT_SOURCE_DIR}/lwjson)
set(FLASHDB_DIR ${PROJECT_SOURCE_DIR}/FlashDB)

set(LD_SCRIPT_RUNTIME ${PROJECT_SOURCE_DIR}/support/gcc_arm_512k.ld)
set(LD_SCRIPT_BOOTLOADER ${PROJECT_SOURCE_DIR}/support/gcc_arm_512k_bootloader.ld)

set(CC_COMMON_FLAGS
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
    -fno-common
    -fstack-usage
    -ffast-math
    -ffat-lto-objects
    -fallow-store-data-races)

set(CXX_COMMON_FLAGS 
    ${CC_COMMON_FLAGS}
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

set(CC_DEBUG_FLAGS
    -O0 -g3)

set(CC_RELEASE_FLAGS   
    -flto=auto
    -Os)

set(LD_COMMON_FLAGS
    -u _printf_float
    -specs=nano.specs
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -lm
    -lnosys
    -Wl,--gc-sections)

set(LD_DEBUG_FLAGS)

set(LD_RELEASE_FLAGS   
    -flto=auto)

set(TARGET_NAME_RUNTIME ${PROJECT_NAME}.elf)
set(HEX_FILE_RUNTIME ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE_RUNTIME ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(DUMP_FILE_RUNTIME ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.S)
set(LD_RUNTIME_FLAGS
    -T${LD_SCRIPT_RUNTIME}
    -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref)

set(TARGET_NAME_BOOTLOADER ${PROJECT_NAME}_bootloader.elf)
set(HEX_FILE_BOOTLOADER ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_bootloader.hex)
set(BIN_FILE_BOOTLOADER ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_bootloader.bin)
set(DUMP_FILE_BOOTLOADER ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_bootloader.S)
set(LD_BOOTLOADER_FLAGS
    -T${LD_SCRIPT_BOOTLOADER}
    -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}_bootloader.map,--cref)

set(COMMON_SOURCES
    ${PROJECT_SOURCE_DIR}/main.c
    ${PROJECT_SOURCE_DIR}/app.cpp
    ${PROJECT_SOURCE_DIR}/network.cpp
    ${PROJECT_SOURCE_DIR}/settingsdb.cpp
    ${PROJECT_SOURCE_DIR}/webserver.cpp
    ${PROJECT_SOURCE_DIR}/support/fal_cfg.c
    ${PROJECT_SOURCE_DIR}/support/stubs.c
    ${PROJECT_SOURCE_DIR}/support/tx_initialize_low_level.c
    ${PROJECT_SOURCE_DIR}/support/emac/synopGMAC_Dev.c
    ${PROJECT_SOURCE_DIR}/support/emac/synopGMAC_network_interface.c
    ${PROJECT_SOURCE_DIR}/support/emac/synopGMAC_plat.c
    ${PROJECT_SOURCE_DIR}/support/emac/nx_m460_eth_driver.c
    ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/GCC/startup_M460.S
    ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/system_m460.c
    ${LWJSON_DIR}/lwjson/src/lwjson/lwjson.c
    ${LWJSON_DIR}/lwjson/src/lwjson/lwjson_stream.c
    ${LWJSON_DIR}/lwjson/src/lwjson/lwjson_debug.c
    ${FLASHDB_DIR}/src/fdb_kvdb.c
    ${FLASHDB_DIR}/src/fdb_utils.c
    ${FLASHDB_DIR}/src/fdb.c
    ${FLASHDB_DIR}/port/fal/src/fal_rtt.c
    ${FLASHDB_DIR}/port/fal/src/fal.c
    ${FLASHDB_DIR}/port/fal/src/fal_partition.c
    ${FLASHDB_DIR}/port/fal/src/fal_flash.c)

add_executable(${TARGET_NAME_RUNTIME} ${COMMON_SOURCES})
add_executable(${TARGET_NAME_BOOTLOADER} ${COMMON_SOURCES})

# Generate mkfat tool locally. This is fragile...
IF (${CMAKE_HOST_SYSTEM_NAME} STREQUAL  "Windows")
    set(MKFAT_BINARY ${PROJECT_SOURCE_DIR}/mkfat/build/Debug/mkfat.exe)
    set(MKFAT_CMAKE_COMMAND cmake -Wno-deprecated -G "Visual Studio 16 2019" .. )
ELSE (${CMAKE_HOST_SYSTEM_NAME} STREQUAL  "Windows")
    set(MKFAT_BINARY ${PROJECT_SOURCE_DIR}/mkfat/build/mkfat)
    set(MKFAT_CMAKE_COMMAND cmake -Wno-deprecated -G "Ninja" .. )
ENDIF (${CMAKE_HOST_SYSTEM_NAME} STREQUAL  "Windows")

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/mkfat/build)
add_custom_command(
    OUTPUT ${MKFAT_BINARY}
    COMMAND ${MKFAT_CMAKE_COMMAND}
    COMMAND cmake --build . 
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/mkfat/build
    DEPENDS ${PROJECT_SOURCE_DIR}/mkfat/main.c)
# Done generating mkfat tool...

# Generate fs.h
file(GLOB VUE3_FILES
       ${PROJECT_SOURCE_DIR}/vue3/src/*.*)
add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/fs.h
    COMMAND yarn build 
    COMMAND ${MKFAT_BINARY} ${PROJECT_SOURCE_DIR}/vue3/dist 512 128 ${PROJECT_BINARY_DIR}/fs.h ${PROJECT_BINARY_DIR}/fs.img
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/vue3
    DEPENDS ${VUE3_FILES} ${MKFAT_BINARY})
target_sources(${TARGET_NAME_RUNTIME} PRIVATE fs.h)

# Generate fsbl.h
file(GLOB FSBL_FILES
       ${PROJECT_SOURCE_DIR}/fsbl/*.*)
add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/fsbl.h
    COMMAND ${MKFAT_BINARY} ${PROJECT_SOURCE_DIR}/fsbl 64 64 ${PROJECT_BINARY_DIR}/fsbl.h ${PROJECT_BINARY_DIR}/fsbl.img
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/fsbl
    DEPENDS ${FSBL_FILES} ${MKFAT_BINARY})
target_sources(${TARGET_NAME_BOOTLOADER} PRIVATE fsbl.h)

# Common include directories
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/support)
include_directories(${PROJECT_SOURCE_DIR}/support/emac)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/m460/Include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${LWJSON_DIR}/lwjson/src/include)
include_directories(${FLASHDB_DIR}/inc)
include_directories(${FLASHDB_DIR}/port/fal/inc)

# StdDriver
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
target_link_libraries(${TARGET_NAME_RUNTIME} standard_driver)
target_link_libraries(${TARGET_NAME_BOOTLOADER} standard_driver)

target_compile_options(standard_driver PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_COMMON_FLAGS}>")
target_compile_options(standard_driver PRIVATE "$<$<CONFIG:DEBUG>:${CC_DEBUG_FLAGS}>")
target_compile_options(standard_driver PRIVATE "$<$<CONFIG:RELEASE>:${CC_RELEASE_FLAGS}>")

# ThreadX
add_subdirectory(${THREADX_DIR})
target_link_libraries(${TARGET_NAME_RUNTIME} threadx)
target_link_libraries(${TARGET_NAME_BOOTLOADER} threadx)

target_compile_options(threadx PRIVATE "$<$<COMPILE_LANGUAGE:ASM>:${CC_COMMON_FLAGS}>")
target_compile_options(threadx PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_COMMON_FLAGS}>")
target_compile_options(threadx PRIVATE "$<$<CONFIG:DEBUG>:${CC_DEBUG_FLAGS}>")
target_compile_options(threadx PRIVATE "$<$<CONFIG:RELEASE>:${CC_RELEASE_FLAGS}>")

# FileX
add_subdirectory(${FILEX_DIR})
target_link_libraries(${TARGET_NAME_RUNTIME} filex)
target_link_libraries(${TARGET_NAME_BOOTLOADER} filex)

target_compile_options(filex PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_COMMON_FLAGS}>")
target_compile_options(filex PRIVATE "$<$<CONFIG:DEBUG>:${CC_DEBUG_FLAGS}>")
target_compile_options(filex PRIVATE "$<$<CONFIG:RELEASE>:${CC_RELEASE_FLAGS}>")

# NetXDuo
add_subdirectory(${NETXDUO_DIR})
target_link_libraries(${TARGET_NAME_RUNTIME} netxduo)
target_link_libraries(${TARGET_NAME_BOOTLOADER} netxduo)

target_compile_options(netxduo PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_COMMON_FLAGS}>")
target_compile_options(netxduo PRIVATE "$<$<CONFIG:DEBUG>:${CC_DEBUG_FLAGS}>")
target_compile_options(netxduo PRIVATE "$<$<CONFIG:RELEASE>:${CC_RELEASE_FLAGS}>")

# Runtime
target_include_directories(${TARGET_NAME_RUNTIME} PRIVATE ${CMAKE_BINARY_DIR})

target_compile_options(${TARGET_NAME_RUNTIME} PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_COMMON_FLAGS}>")
target_compile_options(${TARGET_NAME_RUNTIME} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CXX_COMMON_FLAGS}>")
target_compile_options(${TARGET_NAME_RUNTIME} PRIVATE "$<$<CONFIG:DEBUG>:${CC_DEBUG_FLAGS}>")
target_compile_options(${TARGET_NAME_RUNTIME} PRIVATE "$<$<CONFIG:RELEASE>:${CC_RELEASE_FLAGS}>")

target_link_options(${TARGET_NAME_RUNTIME} PRIVATE ${LD_COMMON_FLAGS})
target_link_options(${TARGET_NAME_RUNTIME} PRIVATE ${LD_RUNTIME_FLAGS})
target_link_options(${TARGET_NAME_RUNTIME} PRIVATE "$<$<CONFIG:DEBUG>:${LD_DEBUG_FLAGS}>")
target_link_options(${TARGET_NAME_RUNTIME} PRIVATE "$<$<CONFIG:RELEASE>:${LD_RELEASE_FLAGS}>")

# Bootloader
target_include_directories(${TARGET_NAME_BOOTLOADER} PRIVATE ${CMAKE_BINARY_DIR})

target_compile_definitions(${TARGET_NAME_BOOTLOADER} PRIVATE BOOTLOADER)

target_compile_options(${TARGET_NAME_BOOTLOADER} PRIVATE "$<$<COMPILE_LANGUAGE:C>:${CC_COMMON_FLAGS}>")
target_compile_options(${TARGET_NAME_BOOTLOADER} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CXX_COMMON_FLAGS}>")
target_compile_options(${TARGET_NAME_BOOTLOADER} PRIVATE "$<$<CONFIG:DEBUG>:${CC_DEBUG_FLAGS}>")
target_compile_options(${TARGET_NAME_BOOTLOADER} PRIVATE "$<$<CONFIG:RELEASE>:${CC_RELEASE_FLAGS}>")

target_link_options(${TARGET_NAME_BOOTLOADER} PRIVATE ${LD_COMMON_FLAGS})
target_link_options(${TARGET_NAME_BOOTLOADER} PRIVATE ${LD_BOOTLOADER_FLAGS})
target_link_options(${TARGET_NAME_BOOTLOADER} PRIVATE "$<$<CONFIG:DEBUG>:${LD_DEBUG_FLAGS}>")
target_link_options(${TARGET_NAME_BOOTLOADER} PRIVATE "$<$<CONFIG:RELEASE>:${LD_RELEASE_FLAGS}>")

# Generate version.h
find_package(Git)

execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_SHORT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${GIT_EXECUTABLE} rev-list HEAD --count WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_REV_COUNT OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${GIT_EXECUTABLE} show -s --format=%ad HEAD --date=iso-strict WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_COMMIT_DATE OUTPUT_STRIP_TRAILING_WHITESPACE)

configure_file("${PROJECT_SOURCE_DIR}/version.h.in" "${CMAKE_BINARY_DIR}/version.h" @ONLY)

add_custom_command(TARGET ${TARGET_NAME_RUNTIME} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -D $<TARGET_FILE:${TARGET_NAME_RUNTIME}> > ${DUMP_FILE_RUNTIME}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME_RUNTIME}> ${BIN_FILE_RUNTIME}
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signature $<TARGET_FILE:${TARGET_NAME_RUNTIME}> ${HEX_FILE_RUNTIME}
    COMMAND ${CMAKE_SIZE} ${TARGET_NAME_RUNTIME}
    COMMENT ("Building ${HEX_FILE_RUNTIME}\\nBuilding ${BIN_FILE_RUNTIME}"))

add_custom_command(TARGET ${TARGET_NAME_BOOTLOADER} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -D $<TARGET_FILE:${TARGET_NAME_BOOTLOADER}> > ${DUMP_FILE_BOOTLOADER}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME_BOOTLOADER}> ${BIN_FILE_BOOTLOADER}
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signature $<TARGET_FILE:${TARGET_NAME_BOOTLOADER}> ${HEX_FILE_BOOTLOADER}
    COMMAND ${CMAKE_SIZE} ${TARGET_NAME_BOOTLOADER}
    COMMENT ("Building ${HEX_FILE_BOOTLOADER}\\nBuilding ${BIN_FILE_BOOTLOADER}"))

set(PROGRAM_CMD "./openocd -f ./stlink.cfg -f ./numicro_m4.cfg -c \"adapter speed 12000\" -c \"program ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin ${BASE_ADDRESS} verify reset exit\"")
install(CODE "execute_process(COMMAND ${PROGRAM_CMD} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/openocd)")
