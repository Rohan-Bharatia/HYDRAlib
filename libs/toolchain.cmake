set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_LINKER arm-none-eabi-ld)
set(CMAKE_C_FLAGS "-mcpu=cortex-m3 -mthumb")
set(CMAKE_CXX_FLAGS "-mcpu=cortex-m3 -mthumb")
set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_SOURCE_DIR}/libpros/pros.ld")
set(CMAKE_MAKE_PROGRAM make)
