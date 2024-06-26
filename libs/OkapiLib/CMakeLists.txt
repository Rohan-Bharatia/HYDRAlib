cmake_minimum_required(VERSION 3.6)
project(OkapiLibV5)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
if(CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    setup_target_for_coverage(${PROJECT_NAME}_coverage tests coverage)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++17 -Wall -Wextra -Wshadow -Wnull-dereference -Wno-psabi -Wno-unused-function -pthread -g -O0 -fprofile-arcs -ftest-coverage -D THREADS_STD")

enable_testing()

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in external-dependencies/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external-dependencies )
if(result)
    message(FATAL_ERROR "CMake step for dependencies failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external-dependencies )
if(result)
    message(FATAL_ERROR "Build step for dependencies failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

add_subdirectory(${CMAKE_BINARY_DIR}/squiggles-src
                 ${CMAKE_BINARY_DIR}/squiggles-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include")
    include_directories("${squiggles_SOURCE_DIR}/include")
endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

include_directories(${CMAKE_BINARY_DIR}/squiggles-src/main/include)
include_directories(include)

add_executable(OkapiLibV5
        include/okapi/api/chassis/controller/chassisController.hpp
        include/okapi/api/chassis/controller/chassisControllerIntegrated.hpp
        include/okapi/api/chassis/controller/chassisControllerPid.hpp
        include/okapi/api/chassis/controller/chassisScales.hpp
        include/okapi/api/chassis/controller/odomChassisController.hpp
        include/okapi/api/chassis/controller/defaultOdomChassisController.hpp
        include/okapi/api/chassis/model/chassisModel.hpp
        include/okapi/api/chassis/model/hDriveModel.hpp
        include/okapi/api/chassis/model/readOnlyChassisModel.hpp
        include/okapi/api/chassis/model/skidSteerModel.hpp
        include/okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp
        include/okapi/api/chassis/model/threeEncoderXDriveModel.hpp
        include/okapi/api/chassis/model/xDriveModel.hpp
        include/okapi/api/control/async/asyncController.hpp
        include/okapi/api/control/async/asyncLinearMotionProfileController.hpp
        include/okapi/api/control/async/asyncMotionProfileController.hpp
        include/okapi/api/control/async/asyncPosIntegratedController.hpp
        include/okapi/api/control/async/asyncPositionController.hpp
        include/okapi/api/control/async/asyncPosPidController.hpp
        include/okapi/api/control/async/asyncVelIntegratedController.hpp
        include/okapi/api/control/async/asyncVelocityController.hpp
        include/okapi/api/control/async/asyncVelPidController.hpp
        include/okapi/api/control/async/asyncWrapper.hpp
        include/okapi/api/control/iterative/iterativeController.hpp
        include/okapi/api/control/iterative/iterativeMotorVelocityController.hpp
        include/okapi/api/control/iterative/iterativePositionController.hpp
        include/okapi/api/control/iterative/iterativePosPidController.hpp
        include/okapi/api/control/iterative/iterativeVelocityController.hpp
        include/okapi/api/control/iterative/iterativeVelPidController.hpp
        include/okapi/api/control/util/controllerRunner.hpp
        include/okapi/api/control/util/flywheelSimulator.hpp
        include/okapi/api/control/util/pathfinderUtil.hpp
        include/okapi/api/control/util/pidTuner.hpp
        include/okapi/api/control/util/settledUtil.hpp
        include/okapi/api/control/closedLoopController.hpp
        include/okapi/api/control/controllerInput.hpp
        include/okapi/api/control/controllerOutput.hpp
        include/okapi/api/control/offsettableControllerInput.hpp
        include/okapi/api/device/button/abstractButton.hpp
        include/okapi/api/device/button/buttonBase.hpp
        include/okapi/api/device/motor/abstractMotor.hpp
        include/okapi/api/device/rotarysensor/continuousRotarySensor.hpp
        include/okapi/api/device/rotarysensor/rotarySensor.hpp
        include/okapi/api/filter/averageFilter.hpp
        include/okapi/api/filter/composableFilter.hpp
        include/okapi/api/filter/demaFilter.hpp
        include/okapi/api/filter/ekfFilter.hpp
        include/okapi/api/filter/emaFilter.hpp
        include/okapi/api/filter/filter.hpp
        include/okapi/api/filter/filteredControllerInput.hpp
        include/okapi/api/filter/medianFilter.hpp
        include/okapi/api/filter/passthroughFilter.hpp
        include/okapi/api/filter/velMath.hpp
        include/okapi/api/odometry/odometry.hpp
        include/okapi/api/odometry/twoEncoderOdometry.hpp
        include/okapi/api/odometry/odomMath.hpp
        include/okapi/api/odometry/threeEncoderOdometry.hpp
        include/okapi/api/units/QAcceleration.hpp
        include/okapi/api/units/QAngle.hpp
        include/okapi/api/units/QAngularAcceleration.hpp
        include/okapi/api/units/QAngularJerk.hpp
        include/okapi/api/units/QAngularSpeed.hpp
        include/okapi/api/units/QArea.hpp
        include/okapi/api/units/QForce.hpp
        include/okapi/api/units/QFrequency.hpp
        include/okapi/api/units/QJerk.hpp
        include/okapi/api/units/QLength.hpp
        include/okapi/api/units/QMass.hpp
        include/okapi/api/units/QPressure.hpp
        include/okapi/api/units/QSpeed.hpp
        include/okapi/api/units/QTime.hpp
        include/okapi/api/units/QTorque.hpp
        include/okapi/api/units/QVolume.hpp
        include/okapi/api/units/RQuantity.hpp
        include/okapi/api/util/abstractRate.hpp
        include/okapi/api/util/logging.hpp
        include/okapi/api/util/timeUtil.hpp
        include/okapi/api/util/abstractTimer.hpp
        include/okapi/api/util/mathUtil.hpp
        include/okapi/api/util/supplier.hpp
        include/okapi/api/coreProsAPI.hpp
        include/test/tests/api/implMocks.hpp
        src/api/chassis/controller/chassisControllerIntegrated.cpp
        src/api/chassis/controller/chassisControllerPid.cpp
        src/api/chassis/controller/chassisScales.cpp
        src/api/chassis/controller/chassisScales.cpp
        src/api/chassis/controller/odomChassisController.cpp
        src/api/chassis/controller/defaultOdomChassisController.cpp
        src/api/chassis/model/hDriveModel.cpp
        src/api/chassis/model/skidSteerModel.cpp
        src/api/chassis/model/threeEncoderSkidSteerModel.cpp
        src/api/chassis/model/threeEncoderXDriveModel.cpp
        src/api/chassis/model/xDriveModel.cpp
        src/api/control/async/asyncLinearMotionProfileController.cpp
        src/api/control/async/asyncMotionProfileController.cpp
        src/api/control/async/asyncPosIntegratedController.cpp
        src/api/control/async/asyncPosPidController.cpp
        src/api/control/async/asyncVelIntegratedController.cpp
        src/api/control/async/asyncVelPidController.cpp
        src/api/control/iterative/iterativeMotorVelocityController.cpp
        src/api/control/iterative/iterativePosPidController.cpp
        src/api/control/iterative/iterativeVelPidController.cpp
        src/api/control/util/flywheelSimulator.cpp
        src/api/control/offsettableControllerInput.cpp
        src/api/control/util/pidTuner.cpp
        src/api/control/util/settledUtil.cpp
        src/api/device/button/abstractButton.cpp
        src/api/device/button/buttonBase.cpp
        src/api/device/motor/abstractMotor.cpp
        src/api/device/rotarysensor/rotarySensor.cpp
        src/api/filter/composableFilter.cpp
        src/api/filter/demaFilter.cpp
        src/api/filter/ekfFilter.cpp
        src/api/filter/emaFilter.cpp
        src/api/filter/filter.cpp
        src/api/filter/passthroughFilter.cpp
        src/api/filter/velMath.cpp
        src/api/odometry/twoEncoderOdometry.cpp
        src/api/odometry/odomMath.cpp
        src/api/odometry/threeEncoderOdometry.cpp
        src/api/util/abstractRate.cpp
        src/api/util/abstractTimer.cpp
        src/api/util/logging.cpp
        src/api/util/timeUtil.cpp
        test/buttonTests.cpp
        test/controllerTests.cpp
        test/controlTests.cpp
        test/filterTests.cpp
        test/hDriveModelTests.cpp
        test/implMocks.cpp
        test/twoEncoderOdometryTests.cpp
        test/utilTests.cpp
        test/unitTests.cpp
        test/loggerTests.cpp
        test/skidSteerModelTests.cpp
        test/xDriveModelTests.cpp
        test/threeEncoderSkidSteerModelTests.cpp
        test/chassisControllerIntegratedTests.cpp
        test/chassisControllerPidTest.cpp
        test/chassisScalesTests.cpp
        test/asyncPosIntegratedControllerTests.cpp
        test/asyncVelIntegratedControllerTests.cpp
        test/asyncVelPIDControllerTests.cpp
        test/asyncMotionProfileControllerTests.cpp
        test/asyncLinearMotionProfileControllerTests.cpp
        test/iterativeVelPIDControllerTests.cpp
        test/iterativeMotorVelocityControllerTest.cpp
        test/iterativePosPIDControllerTests.cpp
        test/defaultOdomChassisControllerTest.cpp
        test/asyncWrapperTests.cpp
        test/offsettableControllerInputTests.cpp
        test/asyncPosPIDControllerTests.cpp
        test/threeEncoderOdometryTests.cpp
        include/okapi/api/odometry/point.hpp
        test/odomMathTests.cpp
        include/okapi/api/odometry/stateMode.hpp
        include/okapi/api/odometry/odomState.hpp
        src/api/odometry/odomState.cpp
        test/threeEncoderXDriveModelTests.cpp)

# Link against gtest
target_link_libraries(OkapiLibV5 gtest_main squiggles)
