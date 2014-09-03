cmake_minimum_required(VERSION 2.8.3)
project(rtt_control_components)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
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
  eigen
  )

include_directories(${catkin_INCLUDE_DIRS})

include_directories( 
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
	include ${scl_filters_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

orocos_install_headers(
	DIRECTORY include/math
	include/arm 
	include/discontinuities  
	include/filters
	include/signalrouting
	include/sources
	include/supervisory
	include/tracing
	include/SOEMinterfacing
	include/ROSinterfacing
	)

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros eigen
)   

### Orocos Components - SIGNALROUTING ###
orocos_component(Demux src/signalrouting/Demux.cpp)
orocos_component(Mux src/signalrouting/Mux.cpp)
orocos_component(Switch src/signalrouting/Switch.cpp)
orocos_component(VectorConcatenate src/signalrouting/VectorConcatenate.cpp)
target_link_libraries(Demux ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Mux ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Switch ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(VectorConcatenate ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - DISCONTINUITY ###
orocos_component(Deadzone src/discontinuities/Deadzone.cpp)
orocos_component(Saturation src/discontinuities/Saturation.cpp)
target_link_libraries(Deadzone ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Saturation ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - MATH ###
orocos_component(MatrixTransform src/math/MatrixTransform.cpp)
orocos_component(Equal src/math/Equal.cpp)
orocos_component(Abs src/math/Abs.cpp)
orocos_component(AddConstant src/math/AddConstant.cpp)
orocos_component(Addition src/math/Addition.cpp)
orocos_component(Derivative src/math/Derivative.cpp)
orocos_component(Differentiate src/math/Differentiate.cpp)
orocos_component(Gain src/math/Gain.cpp)
orocos_component(Integrator src/math/Integrator.cpp)
orocos_component(RealTimeDerivator src/math/RealTimeDerivator.cpp)
orocos_component(Sign src/math/Sign.cpp)
orocos_component(Sqrt src/math/Sqrt.cpp)
orocos_component(Subtraction src/math/Subtraction.cpp)
orocos_component(Sum src/math/Sum.cpp)
#orocos_component(DotProduct src/math/DotProduct.cpp)
#orocos_component(Product src/math/Product.cpp)
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
target_link_libraries(Sqrt ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Subtraction ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Sum ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - SUPERVISORY ###
orocos_component(ActuatorEnabler src/supervisory/ActuatorEnabler.cpp)
orocos_component(Homing src/supervisory/Homing.cpp)
orocos_component(SafetyMonitor src/supervisory/SafetyMonitor.cpp)
orocos_component(Supervisor src/supervisory/Supervisor.cpp)
target_link_libraries(Supervisor ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ActuatorEnabler ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Homing ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(SafetyMonitor ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - Sources ###
orocos_component(ConstantBool src/sources/ConstantBool.cpp)
orocos_component(ConstantSignal src/sources/ConstantSignal.cpp)
orocos_component(GaussianRandomNumber src/sources/GaussianRandomNumber.cpp)
orocos_component(PulseSignal src/sources/PulseSignal.cpp)
orocos_component(RampSignal src/sources/RampSignal.cpp)
orocos_component(SawtoothSignal src/sources/SawtoothSignal.cpp)
orocos_component(SineWave src/sources/SineWave.cpp)
orocos_component(StepSignal src/sources/StepSignal.cpp)
orocos_component(UniformRandomNumber src/sources/UniformRandomNumber.cpp)
orocos_component(ReferenceGenerator_rtt src/sources/ReferenceGenerator.cpp)
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
orocos_component(SkewedNotch src/filters/SkewedNotch.cpp)
orocos_component(AdmittanceControllerSpindle src/filters/AdmittanceControllerSpindle.cpp)
orocos_component(FirstOrderLowPass src/filters/FirstOrderLowPass.cpp)
orocos_component(WeakIntegrator src/filters/WeakIntegrator.cpp)
orocos_component(PID src/filters/PID.cpp)
orocos_component(Polynomial src/filters/Polynomial.cpp)
orocos_component(Controller src/filters/Controller.cpp)
orocos_component(AdmittanceController src/filters/AdmittanceController.cpp)
orocos_component(SecondOrderLowPass src/filters/SecondOrderLowPass.cpp)
orocos_component(LeadLag src/filters/LeadLag.cpp)
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

### Orocos Components - ARM ###
orocos_component(ARM_GravityTorques src/arm/GravityTorques.cpp)
orocos_component(ARM_GravityTorquesParser src/arm/GravityTorquesParser.cpp)
orocos_component(ARM_GripperControl src/arm/GripperControl.cpp)
orocos_component(ARM_TorqueSensorsAutoC3 src/arm/TorqueSensorsAutoC3.cpp)
orocos_component(ARM_TorqueSensors src/arm/TorqueSensors.cpp)
target_link_libraries(ARM_GravityTorques ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ARM_GravityTorquesParser ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ARM_GripperControl ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ARM_TorqueSensorsAutoC3 ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ARM_TorqueSensors ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - ROS_interfacing ###
orocos_component(BenchmarkReference src/ROSinterfacing/BenchmarkReference.cpp)
orocos_component(BoolToROS src/ROSinterfacing/BoolToROS.cpp)
orocos_component(DoublesToJointState src/ROSinterfacing/DoublesToJointState.cpp)
orocos_component(DoublesToROS src/ROSinterfacing/DoublesToROS.cpp)
orocos_component(FollowJointTrajectoryActionToDoubles src/ROSinterfacing/FollowJointTrajectoryActionToDoubles.cpp)
orocos_component(JointStateAggregator src/ROSinterfacing/JointStateAggregator.cpp)
orocos_component(JointStateToDoubles src/ROSinterfacing/JointStateToDoubles.cpp)
orocos_component(JointTrajectoryToDoubles src/ROSinterfacing/JointTrajectoryToDoubles.cpp)
orocos_component(PublishOdometry src/ROSinterfacing/PublishOdometry.cpp)
orocos_component(ReadTwistMsg src/ROSinterfacing/ReadTwistMsg.cpp)
orocos_component(RosDiagnostics src/ROSinterfacing/RosDiagnostics.cpp)
target_link_libraries(BenchmarkReference ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(BoolToROS ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(DoublesToJointState ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(DoublesToROS ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(FollowJointTrajectoryActionToDoubles ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointStateAggregator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointStateToDoubles ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(JointTrajectoryToDoubles ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(PublishOdometry ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReadTwistMsg ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(RosDiagnostics ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Components - soem_interfacing ###
orocos_component(AnalogInsGeneric src/SOEMinterfacing/AnalogInsGeneric.cpp)
orocos_component(AnalogOutsGeneric src/SOEMinterfacing/AnalogOutsGeneric.cpp)
orocos_component(DigitalIns src/SOEMinterfacing/DigitalIns.cpp)
orocos_component(DigitalOuts src/SOEMinterfacing/DigitalOuts.cpp)
orocos_component(ReadEncoders src/SOEMinterfacing/ReadEncoders.cpp)
orocos_component(ReadEncodersTimeStamp src/SOEMinterfacing/ReadEncodersTimeStamp.cpp)
orocos_component(AnalogIns src/SOEMinterfacing/AnalogIns.cpp)
orocos_component(AnalogOuts src/SOEMinterfacing/AnalogOuts.cpp)
orocos_component(AnalogInsPera src/SOEMinterfacing/AnalogInsPera.cpp)
orocos_component(AnalogInsPera2 src/SOEMinterfacing/AnalogInsPera2.cpp)
orocos_component(AnalogOutsPera src/SOEMinterfacing/AnalogOutsPera.cpp)
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
orocos_component(Reporter src/tracing/Reporter.cpp)
orocos_component(ReporterTrigger src/tracing/ReporterTrigger.cpp)
orocos_component(Tracing src/tracing/Tracing.cpp)
orocos_component(TracingOneSignal src/tracing/TracingOneSignal.cpp)
orocos_component(TracingTimeStamp src/tracing/TracingTimeStamp.cpp)
target_link_libraries(Reporter ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(ReporterTrigger ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(Tracing ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TracingOneSignal ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(TracingTimeStamp ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

