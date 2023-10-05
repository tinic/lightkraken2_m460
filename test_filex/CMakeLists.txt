cmake_minimum_required(VERSION 3.27)
project(test)
set(THREADX_ARCH linux)
set(THREADX_TOOLCHAIN gnu)
set(FX_STANDALONE_ENABLE ON)
add_compile_definitions(
    FX_STANDALONE_ENABLE
    FX_ENABLE_EXFAT)
add_compile_options(
    -fsanitize=address
)
add_link_options(
    -fsanitize=address
)
add_executable(test main.c)
add_subdirectory(${PROJECT_SOURCE_DIR}/filex)
target_link_libraries(test filex)
