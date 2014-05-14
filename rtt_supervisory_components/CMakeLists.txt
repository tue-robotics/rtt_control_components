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
orocos_component(Supervisor src/Supervisor.cpp)
orocos_component(SupervisorOld src/SupervisorOld.cpp)
#orocos_component(Homing src/Homing.cpp)
#orocos_component(Safety src/Safety.cpp)
orocos_component(ActuatorEnabler src/ActuatorEnabler.cpp)
orocos_component(SafetyMonitor src/SafetyMonitor.cpp)


#rosbuild_include(rtt_rosnode GenerateRTTtypekit)

#ros_generate_rtt_typekit(soem_beckhoff_drivers)

#orocos_plugin(soem_armEthercat ${soem_beckhoff_drivers_PACKAGE_PATH}/src/soem_beckhoff_drivers.cpp src/soem_armEthercat.cpp)
