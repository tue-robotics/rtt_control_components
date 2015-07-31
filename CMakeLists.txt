cmake_minimum_required(VERSION 2.8.3)
project(rtt_control_components)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
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
  )

include_directories(
	${catkin_INCLUDE_DIRS} 
	${scl_filters_INCLUDE_DIRS} 
	${USE_OROCOS_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS})


orocos_install_headers( DIRECTORY )

file(GLOB_RECURSE HEADER_FILES include/*.hpp)

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)   

#### Orocos Components ###
orocos_component(Supervisor 				src/components/Supervisor.cpp 				${HEADER_FILES})
orocos_component(EtherCATread 				src/components/EtherCATread.cpp 			${HEADER_FILES})
orocos_component(Controller 				src/components/Controller.cpp 				${HEADER_FILES})
orocos_component(Safety 					src/components/Safety.cpp 					${HEADER_FILES})
orocos_component(Homing 					src/components/Homing.cpp 					${HEADER_FILES})
orocos_component(ActuatorEnabler 			src/components/ActuatorEnabler.cpp 			${HEADER_FILES})
orocos_component(DynamixelController 		src/components/DynamixelController.cpp 		${HEADER_FILES})
orocos_component(GripperController 			src/components/GripperController.cpp		${HEADER_FILES})
orocos_component(RgbController 				src/components/RgbController.cpp 			${HEADER_FILES})

target_link_libraries(Supervisor 			${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(EtherCATread 			${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(Controller 			${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(Safety 				${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(Homing 				${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(ActuatorEnabler 		${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(DynamixelController 	${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(GripperController 	${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})
target_link_libraries(RgbController 		${catkin_LIBRARIES} 						${USE_OROCOS_LIBRARIES})

### Orocos Components - TEMP ###
orocos_component(ConstantBool src/sources/ConstantBool.cpp ${HEADER_FILES})
orocos_component(ConstantSignal src/sources/ConstantSignal.cpp ${HEADER_FILES})
orocos_component(GaussianRandomNumber src/sources/GaussianRandomNumber.cpp ${HEADER_FILES})
orocos_component(PulseSignal src/sources/PulseSignal.cpp ${HEADER_FILES})
orocos_component(RampSignal src/sources/RampSignal.cpp ${HEADER_FILES})
orocos_component(SawtoothSignal src/sources/SawtoothSignal.cpp ${HEADER_FILES})
orocos_component(SineWave src/sources/SineWave.cpp ${HEADER_FILES})
orocos_component(StepSignal src/sources/StepSignal.cpp ${HEADER_FILES})
orocos_component(UniformRandomNumber src/sources/UniformRandomNumber.cpp ${HEADER_FILES})
orocos_component(ReferenceGenerator_rtt src/sources/ReferenceGenerator.cpp ${HEADER_FILES})
orocos_component(GlobalReferenceGenerator src/sources/GlobalReferenceGenerator.cpp ${HEADER_FILES})
orocos_component(BenchmarkReference src/sources/BenchmarkReference.cpp ${HEADER_FILES})

target_link_libraries(ConstantBool ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ConstantSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(GaussianRandomNumber ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(PulseSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(RampSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SawtoothSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SineWave ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(StepSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(UniformRandomNumber ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReferenceGenerator_rtt ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(GlobalReferenceGenerator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(BenchmarkReference ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
