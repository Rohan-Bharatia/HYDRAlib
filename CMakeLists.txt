cmake_minimum_required(VERSION 3.29.0)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIED True)
set(CMAKE_CXX_EXTENTIONS False)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

project(HYDRAlib VERSION 1.0.0 LANGUAGES C CXX)

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB BIN "*.a")

add_executable(HYDRAlib ${BIN})

set((CMAKE_TOOLCHAIN_FILE && CMAKE_FIND_ROOT_PATH) "${CMAKE_SOURCE_DIR}/dependencies/toolchain.cmake")

find_package(PKG_CONFIG REQUIRED)
pkg_check_modules(PROS REQUIRED pros)

add_definitions(-D_GNU_SOURCE)
add_definitions(-D_V5RC)
add_definitions(-D_VURC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++23 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu23 -fPIC")
