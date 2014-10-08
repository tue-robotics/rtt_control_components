/** Controller.hpp
 *
 * @class Controller
 *
 * \author Max Baeten
 * \date August, 2014
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Controller.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace FILTERS;

Controller::Controller(const string& name) : 
    TaskContext(name, PreOperational)
{
    // Properties
    addProperty("vector_size",                  vector_size)            .doc("Number of controllers");
    addProperty("gains",                        gains)                  .doc("Gains");
    addProperty("controllers",                  controllers)            .doc("List of used controllers");
    addProperty("sampling_time",                Ts)                     .doc("Sampling time");
    
    // Properties Controllers (fill in only for the used filters)  
    addProperty("zero_freq_WeakIntegrator",     fz_WeakIntegrator)      .doc("zero frequency of the weak integrator");
    addProperty("zero_freq_LeadLag",            fz_LeadLag)             .doc("zero frequency of the lead lag filter");
    addProperty("pole_freq_LeadLag",            fp_LeadLag)             .doc("pole frequency of the lead lag filter");
    addProperty("zero_freq_Notch",         		fz_Notch)               .doc("zero frequency of the Notch filter");
    addProperty("zero_damp_Notch",           	dz_Notch)               .doc("zero damping of the Notch filter");
    addProperty("pole_freq_Notch",         		fp_Notch)               .doc("pole frequency of the Notch filter");
    addProperty("pole_damp_Notch",           	dp_Notch)               .doc("pole damping of the Notch filter");
    addProperty("pole_freq_LowPass",            fp_LowPass)             .doc("pole frequency of the low pass filter");
    addProperty("pole_damp_LowPass",            dp_LowPass)             .doc("pole damping of the low pass filter");

    // Adding ports
    addPort( "ref_in",                          references_inport)      .doc("Control Reference port");
    addEventPort( "pos_in",                     positions_inport)       .doc("Position Port");
    addPort( "enable",                     		enable_inport)  		.doc("Receives enable boolean from safety component, controller sends out zeros if enable = false");
    addPort( "out",                             controleffort_outport)  .doc("Control output port");
    addPort( "jointErrors",                     jointerrors_outport)  	.doc("Joint Errors output port");
}

Controller::~Controller()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters_WeakIntegrator[i];
        delete filters_LeadLag[i];
        delete filters_LowPass[i];
        delete filters_Notch[i];
        filters_WeakIntegrator[i];
        filters_LeadLag[i]= NULL;
        filters_LowPass[i]= NULL;
        filters_Notch[i]= NULL;
    }
}

bool Controller::configureHook()
{
	Logger::In in("Controller::Configure");	
	
	// Determine which controllers are demanded and declare their properties
    WeakIntegrator = false;
    LeadLag = false;
    Notch = false;
    LowPass = false;
    
    for (uint i = 0; i < controllers.size(); i++) {
		if (controllers[i] == "WeakIntegrator") {
			WeakIntegrator = true;
		} else if (controllers[i] == "LeadLag") {
			LeadLag = true;	
		} else if (controllers[i] == "Notch") {
			Notch = true;	
		} else if (controllers[i] == "LowPass") {
			LowPass = true;	
		} else {
			log(Error)<<"Controller: Controller number:" << i << " , " << controllers[i] << " is not supported! Choose from: [WeakIntegrator, LeadLag, Notch, LowPass]!"<<endlog();
		}
	}     

    // create filters
    if (WeakIntegrator) {
        filters_WeakIntegrator.resize(vector_size);
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_WeakIntegrator[i] = new DFILTERS::DWeakIntegrator(fz_WeakIntegrator[i], Ts, 4);
        }
    }
    if (LeadLag) {
        filters_LeadLag.resize(vector_size);
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_LeadLag[i] = new DFILTERS::DLeadLag(fz_LeadLag[i], fp_LeadLag[i], Ts, 4);
        }
    }
    if (Notch) {
        filters_Notch.resize(vector_size);
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_Notch[i] = new DFILTERS::DSkewedNotch(fz_Notch[i], dz_Notch[i], fp_Notch[i], dp_Notch[i], Ts, 4);
        }
    }
    if (LowPass) {
        filters_LowPass.resize(vector_size);
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_LowPass[i] = new DFILTERS::DSecondOrderLowpass(fp_LowPass[i], dp_LowPass[i], Ts);
        }
    }

    // Check input data sizes
    if ( gains.size() != vector_size ) {
        log(Error)<<"Controller:: Wrong size of one of the gains!"<<endlog();
        return false;
    }
    if ( WeakIntegrator && (fz_WeakIntegrator.size() != vector_size )) {
        log(Error)<<"Controller:: Wrong size of one of the weak integrator!"<<endlog();
        return false;
    }
    if ( LeadLag && (fz_LeadLag.size() != vector_size || fp_LeadLag.size() != vector_size )) {
        log(Error)<<"Controller:: Wrong size of one of the lead lag filter!"<<endlog();
        return false;
    }
    if ( Notch && (fz_Notch.size() != vector_size || dz_Notch.size() != vector_size || fp_Notch.size() != vector_size || dp_Notch.size() != vector_size )) {
        log(Error)<<"Controller:: Wrong size of one of the notch filter!"<<endlog();
        return false;
    }
    if ( LowPass && (fp_LowPass.size() != vector_size || dp_LowPass.size() != vector_size )) {
        log(Error)<<"Controller:: Wrong size of one of the low pass!"<<endlog();
        return false;
    }

    // Check input data signs
    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"Controller:: vector_size or Ts parameter is invalid!"<<endlog();
        return false;
    }
    for (uint i = 0; i < vector_size; i++) {
        if (WeakIntegrator) {
            if ( fz_WeakIntegrator[i] < 0.0) {
                log(Error)<<"Controller:: Wrong sign of one of the weak integrator parameters!"<<endlog();
                return false;
            }
        }
        if (LeadLag) {
            if ( fz_LeadLag[i] < 0.0 || fp_LeadLag[i] < 0.0 ) {
                log(Error)<<"Controller:: Wrong sign of one of the leadlag parameters!"<<endlog();
                return false;
            }
        }
        if (Notch) {
            if ( fz_Notch[i] < 0.0 || dz_Notch[i] < 0.0 || fp_Notch[i] < 0.0 || dp_Notch[i] < 0.0) {
                log(Error)<<"Controller:: Wrong sign of one of the notch parameters!"<<endlog();
                return false;
            }
        }
        if (LowPass) {
            if ( fp_LowPass[i] < 0.0 || dp_LowPass[i] < 0.0 ) {
                log(Error)<<"Controller:: Wrong sign of one of the low pass parameters!"<<endlog();
                return false;
            }
        }
    }
	
	return true;
}

bool Controller::startHook()
{
	Logger::In in("Controller::Start");	
	
    // Check validity of Ports:
    if ( !references_inport.connected() || !positions_inport.connected() ) {
        log(Error)<<"Controller: One of the inports is not connected!"<<endlog();
        return false;
    }

    if ( !controleffort_outport.connected() ) {
        log(Error)<<"Controller: Outputport not connected!"<<endlog();
        return false;
    }

    return true;
}

void Controller::updateHook()
{
	Logger::In in("Controller::Update");	
	
    doubles references(vector_size,0.0);
    doubles positions(vector_size,0.0);
    doubles jointErrors(vector_size,0.0);
    doubles output_Gains(vector_size,0.0);
    doubles output_WeakIntegrator(vector_size,0.0);
    doubles output_LeadLag(vector_size,0.0);
    doubles output_Notch(vector_size,0.0);
    doubles output(vector_size,0.0);

    // Read the input ports
    references_inport.read(references);
    positions_inport.read(positions);

	enable_inport.read(safe);
	if (!safe) {
		doubles output(vector_size,0.0);
		controleffort_outport.write(output);
		return;
	}
	else {

		// Compute joint errors
		for (uint i = 0; i < vector_size; i++) {
			jointErrors[i] = references[i]-positions[i];
		}
		// Apply Gain
		for (uint i = 0; i < vector_size; i++) {
			output_Gains[i] = jointErrors[i]*gains[i];
		}
		// Apply Weak Integrator
		if (WeakIntegrator) {
			for (uint i = 0; i < vector_size; i++) {
				filters_WeakIntegrator[i]->update( output_Gains[i] );
				output_WeakIntegrator[i] = filters_WeakIntegrator[i]->getOutput();
			}
		} else {
			for (uint i = 0; i < vector_size; i++) {
				output_WeakIntegrator[i] = output_Gains[i];
			}
		}
		// Apply Lead Lag
		if (LeadLag) {
			for (uint i = 0; i < vector_size; i++) {
				filters_LeadLag[i]->update( output_WeakIntegrator[i] );
				output_LeadLag[i] = filters_LeadLag[i]->getOutput();
			}
		} else {
			for (uint i = 0; i < vector_size; i++) {
				output_LeadLag[i] = output_WeakIntegrator[i];
			}
		}
		// Apply Notch
		if (Notch) {
			for (uint i = 0; i < vector_size; i++) {
				filters_Notch[i]->update( output_LeadLag[i] );
				output_Notch[i] = filters_Notch[i]->getOutput();
			}
		} else {
			for (uint i = 0; i < vector_size; i++) {
				output_Notch[i] = output_LeadLag[i];
			}
		}
		// Apply Low Pass
		if (LowPass) {
			for (uint i = 0; i < vector_size; i++) {
				filters_LowPass[i]->update( output_Notch[i] );
				output[i] = filters_LowPass[i]->getOutput();
			}
		} else {
			for (uint i = 0; i < vector_size; i++) {
				output[i] = output_Notch[i];
			}
		}
			
		// Write the outputs
		controleffort_outport.write(output);
		jointerrors_outport.write(jointErrors);
	}

}

ORO_CREATE_COMPONENT(FILTERS::Controller)
