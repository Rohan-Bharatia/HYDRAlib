cmake_minimum_required(VERSION 3.29.0)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIED True)
set(CMAKE_CXX_EXTENTIONS False)

set(ARCHTUPLE "arm-none-eabi-")
set(DEVICE "VEX EDR V5")

set(MFLAGS "-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -Os -g")
set(CPPFLAGS "-D_POSIX_THREADS -D_UNIX98_THREAD_MUTEX_ATTRIBUTES -D_POSIX_TIMERS -D_POSIX_MONOTONIC_CLOCK")
set(GCCFLAGS "-ffunction-sections -fdata-sections -fdiagnostics-color -funwind-tables")

set(WARNFLAGS "-Wno-psabi")

set(DEPFLAGS "-MT $@ -MMD -MP -MF ${CMAKE_BINARY_DIR}/.d/$*.Td")
set(MAKEDEPFOLDER "-dir -p ${CMAKE_BINARY_DIR}/.d/\$(dir \$(patsubst ${CMAKE_BINARY_DIR}/%, %, ${CMAKE_SOURCE_DIR}/$@))")
set(RENAMEDEPENDENCYFILE "-mv -f ${CMAKE_BINARY_DIR}/.d/$*.Td \$(patsubst ${CMAKE_SOURCE_DIR}/%, ${CMAKE_BINARY_DIR}/.d/%.d, ${CMAKE_SOURCE_DIR}/$<) && touch $@")

set(OBJCOPY "${ARCHTUPLE}objcopy")
set(AR "${ARCHTUPLE}ar")
set(AS "${ARCHTUPLE}gcc")
set(CC "${ARCHTUPLE}gcc")
set(CXX "${ARCHTUPLE}g++")
set(LD "${ARCHTUPLE}g++")
set(SIZETOOL "${ARCHTUPLE}size")
set(READELF "${ARCHTUPLE}readelf")
set(STRIP "${ARCHTUPLE}strip")

set(ASMFLAGS "${MFLAGS} ${WARNFLAGS}")
set(CFLAGS "${MFLAGS} ${CPPFLAGS} ${WARNFLAGS} ${GCCFLAGS} --std=gnu23")
set(CXXFLAGS "${MFLAGS} ${CPPFLAGS} ${WARNFLAGS} ${GCCFLAGS} --std=gnu++23")
set(LDFLAGS "${MFLAGS} ${WARNFLAGS} -nostdlib ${GCCFLAGS}")
set(SIZEFLAGS "-d --common")
set(NUMFMTFLAGS "--to=iec --format %.2f --suffix=B")

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/.d)
file(GLOB_RECURSE ASSET_FILES "static/*")

file(GLOB LIBRARIES "${FWDIR}/*.a")
set(EXCLUDE_COLD_LIBRARIES "${FWDIR}/libc.a" "${FWDIR}/libm.a")
list(REMOVE_ITEM LIBRARIES ${EXCLUDE_COLD_LIBRARIES})
set(COLD_LIBRARIES ${LIBRARIES})
set(LNK_FLAGS "--gc-sections --start-group ${LIBRARIES} -lgcc -lstdc++ --end-group -T${FWDIR}/v5-common.ld")

find_program(NUMFMT NAMES gnumfmt numfmt)
find_program(SED sed)

set(NO_COLOR "\033[0m")
set(OK_COLOR "\033[32;01m")
set(ERROR_COLOR "\033[31;01m")
set(WARN_COLOR "\033[33;01m")
set(STEP_COLOR "\033[37;01m")
set(OK_STRING "${OK_COLOR}[OK]${NO_COLOR}")
set(DONE_STRING "${OK_COLOR}[DONE]${NO_COLOR}")
set(ERROR_STRING "${ERROR_COLOR}[ERRORS]${NO_COLOR}")
set(WARN_STRING "${WARN_COLOR}[WARNINGS]${NO_COLOR}")
set(ECHO "/bin/printf %s\n")
set(ECHO_CMD "@${ECHO} \"\$2\$1${NO_COLOR}\"")
set(ECHON_CMD "@/bin/printf %s \"\$2\$1${NO_COLOR}\"")

set(BUILD_VERBOSE 0)
if(DEFINED VERBOSE)
    set(BUILD_VERBOSE ${VERBOSE})
endif()
if(DEFINED V)
    set(BUILD_VERBOSE ${V})
endif()

if(BUILD_VERBOSE EQUAL 0)
    set(R "@echo")
    set(D "@")
    set(VV "@")
elseif(BUILD_VERBOSE EQUAL 1)
    set(R "@echo")
    set(D "")
    set(VV "@")
elseif(BUILD_VERBOSE EQUAL 2)
    set(R "")
    set(D "")
    set(VV "")
endif()

set(INCLUDE "")
foreach(dir IN LISTS INCDIR EXTRA_INCDIR)
    list(APPEND INCLUDE "-iquote\"${dir}\"")
endforeach()

function(add_asset_target target_name)
    add_custom_command(
        OUTPUT ${target_name}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/static
        COMMAND ${OBJCOPY} -I binary -O elf32-littlearm -B arm ${target_name} ${CMAKE_BINARY_DIR}/static/${target_name}.o
        DEPENDS ${target_name}
        COMMENT "ASSET ${target_name}"
    )
endfunction()

add_executable(my_executable main.cpp ${ASSET_OBJ})
