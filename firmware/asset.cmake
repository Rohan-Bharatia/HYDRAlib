cmake_minimum_required(VERSION 3.29.0)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIED True)
set(CMAKE_CXX_EXTENTIONS False)

project(asset)

set(BINDIR "${CMAKE_BINARY_DIR}/bin")

file(GLOB ASSET_FILES "static/*")

set(ASSET_OBJ)
foreach(file ${ASSET_FILES})
    string(REGEX REPLACE "^static/(.*)$" "${BINDIR}/static/\\1.o" obj_file ${file})
    list(APPEND ASSET_OBJ ${obj_file})
endforeach()

foreach(file ${ASSET_FILES})
    string(REGEX REPLACE "^static/(.*)$" "${BINDIR}/static/\\1.o" obj_file ${file})

    add_custom_command(
        OUTPUT ${obj_file}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${BINDIR}/static"
        COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B arm ${file} ${obj_file}
        DEPENDS ${file}
        COMMENT "ASSET ${obj_file}"
    )
endforeach()

add_custom_target(Assets ALL DEPENDS ${ASSET_OBJ})

add_executable(my_executable main.cpp ${ASSET_OBJ})
