cmake_minimum_required(VERSION 3.29.0)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIED True)
set(CMAKE_CXX_EXTENTIONS False)

project(PROS VERSION 3.8.3)

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES src/*.*)
file(GLOB PROS_LIB "libpros/*.a")

add_executable(PROS ${PROS_LIB})

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/pros-toolchain.cmake")

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROS REQUIRED pros)

add_definitions(-D_GNU_SOURCE)
add_definitions(-DPROS_USE_SIMPLE_NAMES)
add_definitions(-DPROS_USE_LITERALS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu20 -fPIC")
