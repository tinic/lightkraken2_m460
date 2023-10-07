cmake_minimum_required(VERSION 3.0..3.16)
project(mkfat)

set(THREADX_ARCH linux)
set(THREADX_TOOLCHAIN gnu)

set(FX_STANDALONE_ENABLE TRUE)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(FILEX_CUSTOM_PORT ${PROJECT_SOURCE_DIR}/filex_port)

add_compile_definitions(FX_ENABLE_EXFAT)
add_compile_options()
add_link_options()

add_executable(mkfat main.c)
add_subdirectory(${PROJECT_SOURCE_DIR}/../filex ./filex EXCLUDE_FROM_ALL)
target_link_libraries(mkfat filex)
