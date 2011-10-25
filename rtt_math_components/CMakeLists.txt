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
orocos_component(Abs src/Abs.cpp)
orocos_component(AddConstant src/AddConstant.cpp)
orocos_component(Addition src/Addition.cpp)
orocos_component(Derivative src/Derivative.cpp)
orocos_component(Differentiate src/Differentiate.cpp)
#orocos_component(DotProduct src/DotProduct.cpp)
#orocos_component(Product src/Product.cpp)
orocos_component(Sign src/Sign.cpp)
orocos_component(Sqrt src/Sqrt.cpp)
orocos_component(Subtraction src/Subtraction.cpp)
orocos_component(Sum src/Sum.cpp)

