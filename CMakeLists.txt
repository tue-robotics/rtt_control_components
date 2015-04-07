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
	${Eigen_INCLUDE_DIRS})


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

### Orocos Components - SIGNALROUTING ###
orocos_component(Demux src/signalrouting/Demux.cpp ${HEADER_FILES})
orocos_component(Mux src/signalrouting/Mux.cpp ${HEADER_FILES})
orocos_component(Switch src/signalrouting/Switch.cpp ${HEADER_FILES})
orocos_component(VectorConcatenate src/signalrouting/VectorConcatenate.cpp ${HEADER_FILES})
target_link_libraries(Demux ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Mux ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Switch ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(VectorConcatenate ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - DISCONTINUITY ###
orocos_component(Deadzone src/discontinuities/Deadzone.cpp ${HEADER_FILES})
orocos_component(Saturation src/discontinuities/Saturation.cpp ${HEADER_FILES})
target_link_libraries(Deadzone ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Saturation ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - MATH ###
orocos_component(MatrixTransform src/math/MatrixTransform.cpp ${HEADER_FILES})
orocos_component(Equal src/math/Equal.cpp ${HEADER_FILES})
orocos_component(Abs src/math/Abs.cpp ${HEADER_FILES})
orocos_component(AddConstant src/math/AddConstant.cpp ${HEADER_FILES})
orocos_component(Addition src/math/Addition.cpp ${HEADER_FILES})
orocos_component(Derivative src/math/Derivative.cpp ${HEADER_FILES})
orocos_component(Differentiate src/math/Differentiate.cpp ${HEADER_FILES})
orocos_component(Gain src/math/Gain.cpp ${HEADER_FILES})
orocos_component(Integrator src/math/Integrator.cpp ${HEADER_FILES})
orocos_component(RealTimeDerivator src/math/RealTimeDerivator.cpp ${HEADER_FILES})
orocos_component(Sign src/math/Sign.cpp ${HEADER_FILES})
orocos_component(SmallerLarger src/math/SmallerLarger.cpp ${HEADER_FILES})
orocos_component(Sqrt src/math/Sqrt.cpp ${HEADER_FILES})
orocos_component(Subtraction src/math/Subtraction.cpp ${HEADER_FILES})
orocos_component(Sum src/math/Sum.cpp ${HEADER_FILES})
orocos_component(Sine src/math/Sine.cpp ${HEADER_FILES})
#orocos_component(DotProduct src/math/DotProduct.cpp ${HEADER_FILES})
#orocos_component(Product src/math/Product.cpp ${HEADER_FILES})
#target_link_libraries(DotProduct ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
#target_link_libraries(Product ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(MatrixTransform ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Equal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Abs ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AddConstant ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Addition ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Derivative ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Differentiate ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Gain ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Integrator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(RealTimeDerivator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Sign ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SmallerLarger ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Sqrt ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Subtraction ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Sum ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Sine ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - SUPERVISORY ###
orocos_component(ActuatorEnabler src/supervisory/ActuatorEnabler.cpp ${HEADER_FILES})
orocos_component(Homing src/supervisory/Homing.cpp ${HEADER_FILES})
orocos_component(SafetyMonitor src/supervisory/SafetyMonitor.cpp ${HEADER_FILES})
orocos_component(Supervisor src/supervisory/Supervisor.cpp ${HEADER_FILES})
orocos_component(SaturationCheck src/supervisory/SaturationCheck.cpp ${HEADER_FILES})
orocos_component(Safety src/supervisory/Safety.cpp ${HEADER_FILES})
target_link_libraries(Supervisor ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ActuatorEnabler ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Homing ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SafetyMonitor ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SaturationCheck ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Safety ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})


### Orocos Components - Sources ###
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
target_link_libraries(BenchmarkReference ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(GlobalReferenceGenerator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReferenceGenerator_rtt ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ConstantBool ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ConstantSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(GaussianRandomNumber ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(PulseSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(RampSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SawtoothSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SineWave ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(StepSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(UniformRandomNumber ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Filters ###
orocos_component(SkewedNotch src/filters/SkewedNotch.cpp ${HEADER_FILES})
orocos_component(AdmittanceControllerSpindle src/filters/AdmittanceControllerSpindle.cpp ${HEADER_FILES})
orocos_component(FirstOrderLowPass src/filters/FirstOrderLowPass.cpp ${HEADER_FILES})
orocos_component(WeakIntegrator src/filters/WeakIntegrator.cpp ${HEADER_FILES})
orocos_component(PID src/filters/PID.cpp ${HEADER_FILES})
orocos_component(Polynomial src/filters/Polynomial.cpp ${HEADER_FILES})
orocos_component(Controller src/filters/Controller.cpp ${HEADER_FILES})
orocos_component(AdmittanceController src/filters/AdmittanceController.cpp ${HEADER_FILES})
orocos_component(SecondOrderLowPass src/filters/SecondOrderLowPass.cpp ${HEADER_FILES})
orocos_component(LeadLag src/filters/LeadLag.cpp ${HEADER_FILES})
orocos_component(FeedForward src/filters/FeedForward.cpp ${HEADER_FILES})
orocos_component(Current2VoltageFFW src/filters/Current2VoltageFFW.cpp ${HEADER_FILES})
target_link_libraries(SkewedNotch ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AdmittanceControllerSpindle ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(FirstOrderLowPass ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(WeakIntegrator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(PID ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Polynomial ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Controller ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AdmittanceController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SecondOrderLowPass ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(LeadLag ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(FeedForward ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Current2VoltageFFW ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - ARM ###
orocos_component(GravityTorquesManual src/arm/GravityTorquesManual.cpp ${HEADER_FILES})
orocos_component(GravityTorques src/arm/GravityTorques.cpp ${HEADER_FILES})
orocos_component(GripperControl src/arm/GripperControl.cpp ${HEADER_FILES})
orocos_component(TorqueSensorsAutoC3 src/arm/TorqueSensorsAutoC3.cpp ${HEADER_FILES})
orocos_component(TorqueSensors src/arm/TorqueSensors.cpp ${HEADER_FILES})
target_link_libraries(GravityTorquesManual ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(GravityTorques ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(GripperControl ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TorqueSensorsAutoC3 ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TorqueSensors ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - ROS_interfacing ###
orocos_component(BoolToROS src/ROSinterfacing/BoolToROS.cpp ${HEADER_FILES})
orocos_component(DoublesToJointState src/ROSinterfacing/DoublesToJointState.cpp ${HEADER_FILES})
orocos_component(DoublesToROS src/ROSinterfacing/DoublesToROS.cpp ${HEADER_FILES})
orocos_component(TrajectoryActionlib src/ROSinterfacing/TrajectoryActionlib.cpp ${HEADER_FILES})
orocos_component(JointStateAggregator src/ROSinterfacing/JointStateAggregator.cpp ${HEADER_FILES})
orocos_component(JointStateToDoubles src/ROSinterfacing/JointStateToDoubles.cpp ${HEADER_FILES})
orocos_component(JointTrajectoryToDoubles src/ROSinterfacing/JointTrajectoryToDoubles.cpp ${HEADER_FILES})
orocos_component(PublishOdometry src/ROSinterfacing/PublishOdometry.cpp ${HEADER_FILES})
orocos_component(ReadTwistMsg src/ROSinterfacing/ReadTwistMsg.cpp ${HEADER_FILES})
orocos_component(RosDiagnostics src/ROSinterfacing/RosDiagnostics.cpp ${HEADER_FILES})
orocos_component(JointStateDistributor src/ROSinterfacing/JointStateDistributor.cpp ${HEADER_FILES})
target_link_libraries(BoolToROS ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(DoublesToJointState ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(DoublesToROS ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TrajectoryActionlib ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointStateAggregator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointStateToDoubles ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointTrajectoryToDoubles ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(PublishOdometry ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReadTwistMsg ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(RosDiagnostics ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointStateDistributor ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - soem_interfacing ###
orocos_component(AnalogInsGeneric src/SOEMinterfacing/AnalogInsGeneric.cpp ${HEADER_FILES})
orocos_component(AnalogOutsGeneric src/SOEMinterfacing/AnalogOutsGeneric.cpp ${HEADER_FILES})
orocos_component(DigitalIns src/SOEMinterfacing/DigitalIns.cpp ${HEADER_FILES})
orocos_component(DigitalOuts src/SOEMinterfacing/DigitalOuts.cpp ${HEADER_FILES})
orocos_component(ReadEncoders src/SOEMinterfacing/ReadEncoders.cpp ${HEADER_FILES})
orocos_component(ReadEncodersTimeStamp src/SOEMinterfacing/ReadEncodersTimeStamp.cpp ${HEADER_FILES})
orocos_component(AnalogIns src/SOEMinterfacing/AnalogIns.cpp ${HEADER_FILES})
orocos_component(AnalogOuts src/SOEMinterfacing/AnalogOuts.cpp ${HEADER_FILES})
orocos_component(AnalogInsPera src/SOEMinterfacing/AnalogInsPera.cpp ${HEADER_FILES})
orocos_component(AnalogInsPera2 src/SOEMinterfacing/AnalogInsPera2.cpp ${HEADER_FILES})
orocos_component(AnalogOutsPera src/SOEMinterfacing/AnalogOutsPera.cpp ${HEADER_FILES})
target_link_libraries(AnalogInsGeneric ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AnalogOutsGeneric ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(DigitalIns ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(DigitalOuts ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReadEncoders ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReadEncodersTimeStamp ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AnalogIns ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AnalogOuts ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AnalogInsPera ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AnalogInsPera2 ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(AnalogOutsPera ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Tracing ###
orocos_component(Reporter src/tracing/Reporter.cpp ${HEADER_FILES})
orocos_component(ReporterTrigger src/tracing/ReporterTrigger.cpp ${HEADER_FILES})
orocos_component(Tracing src/tracing/Tracing.cpp ${HEADER_FILES})
orocos_component(TracingOneSignal src/tracing/TracingOneSignal.cpp ${HEADER_FILES})
orocos_component(TracingTimeStamp src/tracing/TracingTimeStamp.cpp ${HEADER_FILES})
orocos_component(TraceCrash src/tracing/TraceCrash.cpp ${HEADER_FILES})
target_link_libraries(Reporter ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReporterTrigger ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Tracing ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TracingOneSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TracingTimeStamp ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TraceCrash ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Dynamixel ###
orocos_component(DynamixelController src/dynamixel/DynamixelController.cpp ${HEADER_FILES})
target_link_libraries(DynamixelController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Lighting ###
orocos_component(rgb_controller src/lighting/rgb_controller.cpp ${HEADER_FILES})
target_link_libraries(rgb_controller ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Add dependencies for all components making use of messages inside soem_beckhoff_drivers
add_dependencies(Supervisor soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(DynamixelController soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(DigitalOuts soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogInsPera soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(ReadEncodersTimeStamp soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(ReadEncoders soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogInsGeneric soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogOutsGeneric soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogInsPera2 soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogIns soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogOutsPera soem_beckhoff_drivers_generate_messages_cpp)
add_dependencies(AnalogOuts soem_beckhoff_drivers_generate_messages_cpp)
