#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ARM_TorqueSensorsAutoC3.hpp"

using namespace std;
using namespace RTT;
using namespace ARM;

SensorTorquesAutoC3::SensorTorquesAutoC3(const string& name) : TaskContext(name, PreOperational)
{
    addProperty( "vector_size", Nin).doc("Number of In and Outputs");
    addProperty( "obs_inputs", obs_inputs).doc("obsolete inputs, to specify which inputs are obsolete"); // example array (3, 6) will make sure the third and sixth input are discarded
    addProperty( "c1", c1).doc("Calibration Coefficien c1");
	addProperty( "c2", c2).doc("Calibration Coefficient c2");

	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorquesAutoC3::~SensorTorquesAutoC3(){}

bool SensorTorquesAutoC3::configureHook()
{	
    if (c1.size()!= Nin || c2.size()!= Nin ) {
        log(Error)<<"ARM_TorqueSensors: Could not configure component: Erroneus parameters: c1, c2 or c3."<< endlog();
        return false;
    }
    if (obs_inputs.size() < Nin ) {
        log(Error)<<"ARM_TorqueSensors: Could not configure component: Erroneus parameters: obs_inputs.size() should equal the number of obsolete inputs!"<< endlog();
        return false;
    }
	
    Nout = Nin-obs_inputs.size();
    Vmeasured.assign(0.0,Nin);
    Tmeasured.assign(0.0,Nout);
    
	c3.assign(Nin,0.0);
	Tmean.assign(0.0,Nin);
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
		log(Error)<<"SensorTorques: Could not start component: Motor torques outport not connected!"<<endlog();
		return false;
	}
	return true;
}

void SensorTorquesAutoC3::updateHook()
{
	// Calculate Tmeasured (and clear obsolete inputs)
	voltage_inport.read(Vmeasured);
	    
    unsigned int j =0;
    unsigned int k =0;
    for (unsigned int i=0; i<Nin; i++) {
        if (i != (obs_inputs[k]-1)) {
            j++;
        } else if (k != (Nin-Nout-1)) { // if the ith input is obsolote, then j is not increased but k is increased to check for the next obsolete input. unless k equals Nin-Nout-1. then k is not increased any further
            k++;
        }
		Tmeasured[j] = (c1[i]/(Vmeasured[i] + c2[i])+c3[i]);
	}
	
	// Estimation step, c3 is initialised at zero and in this step c3 is estimated
	if (EstimationComplete == false) {
		// step 1: wait 0.5s and then start calculation of Tmean
		if (cntr >= 500  && EstimationStarted == false) {
			EstimationStarted = true;
			cntr = 0;
		}
		else if (EstimationStarted == true) {
			doubles Tmean_(Nout, 0.0);
			Tmean_ = Tmean;
			for (unsigned int i=0; i<Nin; i++) {
				Tmean[i] = (Tmean_[i]*cntr + Tmeasured[i]) / (cntr + 1);
			}
		}
		
		// step 2: set c3 to negative mean after 1.5 second (0.5s waiting and 1.5s calculating mean)
		if (cntr >= 1000  && EstimationStarted == true) {
			EstimationComplete = true;
			for (unsigned int i=0; i<Nin; i++) {
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
