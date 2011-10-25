cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_find_ros_package(rtt )
find_package(OROCOS-RTT HINTS ${rtt_PACKAGE_PATH}/../install )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#common commands for building c++ executables and libraries
orocos_component(ConstantSignal src/ConstantSignal.cpp)
orocos_component(GaussianRandomNumber src/GaussianRandomNumber.cpp)
orocos_component(PulseSignal src/PulseSignal.cpp)
orocos_component(RampSignal src/RampSignal.cpp)
orocos_component(SawtoothSignal src/SawtoothSignal.cpp)
orocos_component(SineWave src/SineWave.cpp)
orocos_component(StepSignal src/StepSignal.cpp)
orocos_component(UniformRandomNumber src/UniformRandomNumber.cpp)

