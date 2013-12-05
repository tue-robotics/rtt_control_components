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
orocos_component(AddmittanceControllers src/AdmittanceControllers.cpp)
orocos_component(AddmittanceControllersSpindle src/AdmittanceControllersSpindle.cpp)
orocos_component(FirstOrderLowPass src/FirstOrderLowPass.cpp)
orocos_component(LeadLag src/LeadLag.cpp)
orocos_component(LeadLags src/LeadLags.cpp)
orocos_component(PID src/PID.cpp)
orocos_component(Polynomials src/Polynomials.cpp)
orocos_component(SecondOrderLowPass src/SecondOrderLowPass.cpp)
orocos_component(SecondOrderLowPasses src/SecondOrderLowPasses.cpp)
orocos_component(SkewedNotch src/SkewedNotch.cpp)
orocos_component(WeakIntegrator src/WeakIntegrator.cpp)
orocos_component(WeakIntegrators src/WeakIntegrators.cpp)

