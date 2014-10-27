#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "TorqueSensorsAutoC3.hpp"

using namespace std;
using namespace RTT;
using namespace ARM;

SensorTorquesAutoC3::SensorTorquesAutoC3(const string& name) : TaskContext(name, PreOperational)
{
    addProperty( "vector_size", N).doc("Number of In and Outputs");
    addProperty( "c1", c1).doc("Calibration Coefficien c1");
	addProperty( "c2", c2).doc("Calibration Coefficient c2");

	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorquesAutoC3::~SensorTorquesAutoC3(){}

bool SensorTorquesAutoC3::configureHook()
{
		
    if (c1.size()!= N || c2.size()!= N ) {
        log(Error)<<"ARM_TorqueSensors: Could not configure component: Erroneus parameters: c1, c2 or c3."<< endlog();
        return false;
    }
    
    Vmeasured.assign(N,0.0);
    Tmeasured.assign(N,0.0);
	c3.assign(N,0.0);
	Tmean.assign(N,0.0);
	cntr = 0;
	EstimationComplete = false;
	EstimationStarted = false;

	return true;
}

bool SensorTorquesAutoC3::startHook()
{
	
	if (!voltage_inport.connected()) {
		log(Error)<<"SensorTorques: Could not start component: Inputport not connected!"<<endlog();
		return false;
	}
	if (!measured_torques_outport.connected()) {
		log(Warning)<<"Motor torques outport not connected!"<<endlog();
	}
	return true;
}

void SensorTorquesAutoC3::updateHook()
{
	
	voltage_inport.read(Vmeasured);
	    
    for (unsigned int i=0; i<N; i++) {
		Tmeasured[i] = (c1[i]/(Vmeasured[i] + c2[i])+c3[i]);
	}
	
	// Estimation step, c3 is initialised at zero and in this step c3 is estimated
	if (EstimationComplete == false) {
		// step 1: wait 0.5s and then start calculation of Tmean
		if (cntr >= 500  && EstimationStarted == false) {
			EstimationStarted = true;
			cntr = 0;
		}
		else if (EstimationStarted == true) {
			doubles Tmean_(N, 0.0);
			Tmean_ = Tmean;
			for (unsigned int i=0; i<N; i++) {
				Tmean[i] = (Tmean_[i]*cntr + Tmeasured[i]) / (cntr + 1);
			}
		}
		
		// step 2: set c3 to negative mean after 1.5 second (0.5s waiting and 1.5s calculating mean)
		if (cntr >= 1000  && EstimationStarted == true) {
			EstimationComplete = true;
			for (unsigned int i=0; i<N; i++) {
				c3[i] = -1.0*Tmean[i];
			}			
			log(Warning) << "SensorTorquesAutoC3: Calculated c3 coefficients : [" << c3[0] << "," << c3[1] << "," << c3[2] << "," << c3[3] << "," << c3[4] << "," << c3[5] << "," << c3[6] << "," << c3[7] << "]" <<endlog();
		}		
		cntr++;
	}
	else {
		measured_torques_outport.write(Tmeasured);
	}
}

ORO_CREATE_COMPONENT(ARM::SensorTorquesAutoC3)
