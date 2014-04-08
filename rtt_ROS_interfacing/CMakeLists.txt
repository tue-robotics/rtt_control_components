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
orocos_component(BoolToROS src/BoolToROS.cpp)
orocos_component(DoubleToROS src/DoubleToROS.cpp)
orocos_component(DoublesToROS src/DoublesToROS.cpp)
orocos_component(DoublesToJointState src/DoublesToJointState.cpp)
orocos_component(JointStateAggregator src/JointStateAggregator.cpp)
orocos_component(PublishOdometry src/PublishOdometry.cpp)
orocos_component(ReadTwistMsg src/ReadTwistMsg.cpp)
orocos_component(RosDiagnostics src/RosDiagnostics.cpp)
orocos_component(JointStateToDoubles src/JointStateToDoubles.cpp)
orocos_component(JointTrajectoryToDoubles src/JointTrajectoryToDoubles.cpp)
