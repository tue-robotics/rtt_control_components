#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "TorqueSensors.hpp"

using namespace std;
using namespace RTT;
using namespace ARM;

SensorTorques::SensorTorques(const string& name) : TaskContext(name, PreOperational)
{
    addProperty( "vector_size", N).doc("Number of In and Outputs");
    addProperty( "c1", c1).doc("Calibration Coefficien c1");
	addProperty( "c2", c2).doc("Calibration Coefficient c2");
	addProperty( "c3", c3).doc("Calibration Coefficient c3");

	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorques::~SensorTorques(){}

bool SensorTorques::configureHook()
{	
	Logger::In in("SensorTorques::Configure");	
	
    if (c1.size()!= N || c2.size()!= N || c3.size()!= N ) {
        log(Error)<<"ARM_TorqueSensors: Could not configure component: Erroneus parameters: c1, c2 or c3."<< endlog();
        return false;
    }
    
    Vmeasured.assign(N,0.0);
    Tmeasured.assign(N,0.0);

	return true;
}

bool SensorTorques::startHook()
{
	Logger::In in("SensorTorques::Start");	
		
	if (!voltage_inport.connected()) {
		log(Error)<<"SensorTorques: Could not start component: Inputport not connected!"<<endlog();
		return false;
	}
	if (!measured_torques_outport.connected()) {
		log(Warning)<<"Motor torques outport not connected!"<<endlog();
	}
	return true;
}

void SensorTorques::updateHook()
{
	Logger::In in("SensorTorques::Update");	
	
	voltage_inport.read(Vmeasured);
	
	for ( uint i = 0; i < N; i++ ) {
        Tmeasured[i] = (c1[i]/(Vmeasured[i] + c2[i])+c3[i]);
	}
	
	measured_torques_outport.write(Tmeasured);
}

ORO_CREATE_COMPONENT(ARM::SensorTorques)
