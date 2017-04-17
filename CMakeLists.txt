cmake_minimum_required(VERSION 2.8.3)
project(rtt_control_components)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  std_msgs
  diagnostic_msgs
  amigo_ref_interpolator
  scl_filters
  roscpp
  tf
  kdl_parser
  rtt_std_msgs
  tue_manipulation
  soem_beckhoff_drivers
  )

include_directories(
	include
	include/math
	include/arm 
	include/discontinuities  
	include/filters
	include/signalrouting
	include/sources
	include/supervisory
	include/tracing
	include/SOEMinterfacing
	include/ROSinterfacing
	include/dynamixel
	include/lighting
	${catkin_INCLUDE_DIRS} 
	${scl_filters_INCLUDE_DIRS} 
	${USE_OROCOS_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS})


orocos_install_headers( DIRECTORY
	include
	include/math
	include/arm 
	include/discontinuities  
	include/filters
	include/signalrouting
	include/sources
	include/supervisory
	include/tracing
	include/SOEMinterfacing
	include/ROSinterfacing
	include/dynamixel
	include/lighting)

file(GLOB_RECURSE HEADER_FILES include/*.hpp)

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)   

### Orocos Components - New ###
orocos_component(EtherCATread src/newcomponents/EtherCATread.cpp ${HEADER_FILES})
orocos_component(EtherCATwrite src/newcomponents/EtherCATwrite.cpp ${HEADER_FILES})
target_link_libraries(EtherCATread ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(EtherCATwrite ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - SUPERVISORY ###
orocos_component(Homing src/supervisory/Homing.cpp ${HEADER_FILES})
target_link_libraries(Homing ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(Supervisor src/supervisory/Supervisor.cpp ${HEADER_FILES})
target_link_libraries(Supervisor ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(Safety src/supervisory/Safety.cpp ${HEADER_FILES})
target_link_libraries(Safety ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Filters ###
orocos_component(Polynomial src/filters/Polynomial.cpp ${HEADER_FILES})
target_link_libraries(Polynomial ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(Controller src/filters/Controller.cpp ${HEADER_FILES})
target_link_libraries(Controller ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(SecondOrderLowPass src/filters/SecondOrderLowPass.cpp ${HEADER_FILES})
target_link_libraries(SecondOrderLowPass ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(LeadLag src/filters/LeadLag.cpp ${HEADER_FILES})
target_link_libraries(LeadLag ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(FeedForward src/filters/FeedForward.cpp ${HEADER_FILES})
target_link_libraries(FeedForward ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(Current2VoltageFFW src/filters/Current2VoltageFFW.cpp ${HEADER_FILES})
target_link_libraries(Current2VoltageFFW ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - ARM ###
orocos_component(GripperControl src/arm/GripperControl.cpp ${HEADER_FILES})
target_link_libraries(GripperControl ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(HandoverDetector src/arm/HandoverDetector.cpp ${HEADER_FILES})
target_link_libraries(HandoverDetector ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - ROS_interfacing ###
orocos_component(DoublesToJointState src/ROSinterfacing/DoublesToJointState.cpp ${HEADER_FILES})
target_link_libraries(DoublesToJointState ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(DoublesToROS src/ROSinterfacing/DoublesToROS.cpp ${HEADER_FILES})
target_link_libraries(DoublesToROS ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(TrajectoryActionlib src/ROSinterfacing/TrajectoryActionlib.cpp ${HEADER_FILES})
target_link_libraries(TrajectoryActionlib ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(JointStateAggregator src/ROSinterfacing/JointStateAggregator.cpp ${HEADER_FILES})
target_link_libraries(JointStateAggregator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(JointStateToDoubles src/ROSinterfacing/JointStateToDoubles.cpp ${HEADER_FILES})
target_link_libraries(JointStateToDoubles ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(PublishOdometry src/ROSinterfacing/PublishOdometry.cpp ${HEADER_FILES})
target_link_libraries(PublishOdometry ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(ReadTwistMsg src/ROSinterfacing/ReadTwistMsg.cpp ${HEADER_FILES})
target_link_libraries(ReadTwistMsg ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(RosDiagnostics src/ROSinterfacing/RosDiagnostics.cpp ${HEADER_FILES})
target_link_libraries(RosDiagnostics ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - soem_interfacing ###
orocos_component(AnalogInsGeneric src/SOEMinterfacing/AnalogInsGeneric.cpp ${HEADER_FILES})
target_link_libraries(AnalogInsGeneric ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(AnalogOutsGeneric src/SOEMinterfacing/AnalogOutsGeneric.cpp ${HEADER_FILES})
target_link_libraries(AnalogOutsGeneric ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(DigitalOuts src/SOEMinterfacing/DigitalOuts.cpp ${HEADER_FILES})
target_link_libraries(DigitalOuts ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(ReadEncoders src/SOEMinterfacing/ReadEncoders.cpp ${HEADER_FILES})
target_link_libraries(ReadEncoders ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Math ###
orocos_component(SmallerLarger src/math/SmallerLarger.cpp ${HEADER_FILES})
target_link_libraries(SmallerLarger ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(MatrixTransform src/math/MatrixTransform.cpp ${HEADER_FILES})
target_link_libraries(MatrixTransform ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(Gain src/math/Gain.cpp ${HEADER_FILES})
target_link_libraries(Gain ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(Addition src/math/Addition.cpp ${HEADER_FILES})
target_link_libraries(Addition ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Sources ###
orocos_component(ConstantSignal src/sources/ConstantSignal.cpp ${HEADER_FILES})
target_link_libraries(ConstantSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
orocos_component(ConstantBool src/sources/ConstantBool.cpp ${HEADER_FILES})
target_link_libraries(ConstantBool ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Dynamixel ###
orocos_component(DynamixelController src/dynamixel/DynamixelController.cpp ${HEADER_FILES})
target_link_libraries(DynamixelController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Lighting ###
orocos_component(rgb_controller src/lighting/rgb_controller.cpp ${HEADER_FILES})
target_link_libraries(rgb_controller ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Add dependencies for all components making use of messages inside soem_beckhoff_drivers
add_dependencies(Supervisor ${catkin_EXPORTED_TARGETS})
add_dependencies(DynamixelController ${catkin_EXPORTED_TARGETS})
add_dependencies(DigitalOuts ${catkin_EXPORTED_TARGETS})
add_dependencies(ReadEncoders ${catkin_EXPORTED_TARGETS})
add_dependencies(AnalogInsGeneric ${catkin_EXPORTED_TARGETS})
add_dependencies(AnalogOutsGeneric ${catkin_EXPORTED_TARGETS})
add_dependencies(EtherCATread ${catkin_EXPORTED_TARGETS})
add_dependencies(EtherCATwrite ${catkin_EXPORTED_TARGETS})
