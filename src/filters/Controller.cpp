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
    addProperty("number_of_ffwports",           N_ffwinports)     		.doc("The amount of feed forward ports");
    addProperty("controllers",                  controllers)            .doc("List of used controllers");
    addProperty("sampling_time",                Ts)                     .doc("Sampling time");
    addProperty("number_of_refports",           N_refinports)           .doc("Number of inports");
    addProperty("refinport_sizes",              inport_sizes)           .doc("Sizes of inports");

    // Controller Properties (You don't have to fill in the unused filters)
    addProperty("gains",                        gains)                  .doc("Gains");
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
    addEventPort( "pos_in",                     positions_inport)       .doc("Position Port");
    addPort( "enable",                     		enable_inport)  		.doc("Receives enable boolean from safety component, controller sends out zeros if enable = false");
    addPort( "out",                             controleffort_outport)  .doc("Control output port");
    addPort( "jointErrors",                     jointerrors_outport)  	.doc("Joint Errors output port");
}

Controller::~Controller()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        if (WeakIntegrator) {
            delete filters_WeakIntegrator[i];
            filters_WeakIntegrator[i] = NULL;
        }
        if (LeadLag) {
            delete filters_LeadLag[i];
            filters_LeadLag[i]= NULL;
        }
        if (LowPass) {
            delete filters_LowPass[i];
            filters_LowPass[i]= NULL;
        }
        if (Notch) {
            delete filters_Notch[i];
            filters_Notch[i]= NULL;
        }
    }
}

bool Controller::configureHook()
{
    // Add Inports and FFW ports
    for ( uint j = 0; j < N_refinports; j++ ) {
        addPort( ("ref_in"+to_string(j+1)),		references_inport[j] )		.doc("Control Reference port");
    }
    for (uint j = 0; j < N_ffwinports; j++) {
        addPort( ("ffw_in" + to_string(j+1)),	ffw_inport[j])				.doc("One of the FFW inports");
    }

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
            log(Error)<<"Controller: Could not configure component: Controller number:" << i << " , " << controllers[i] << " is not supported! Choose from: [WeakIntegrator, LeadLag, Notch, LowPass]!"<<endlog();
            return false;
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
        log(Error)<<"Controller: Could not configure component: Wrong size of one of the gains!"<<endlog();
        return false;
    }
    if ( WeakIntegrator && (fz_WeakIntegrator.size() != vector_size )) {
        log(Error)<<"Controller: Could not configure component: Wrong size of one of the weak integrator!"<<endlog();
        return false;
    }
    if ( LeadLag && (fz_LeadLag.size() != vector_size || fp_LeadLag.size() != vector_size )) {
        log(Error)<<"Controller: Could not configure component: Wrong size of one of the lead lag filter!"<<endlog();
        return false;
    }
    if ( Notch && (fz_Notch.size() != vector_size || dz_Notch.size() != vector_size || fp_Notch.size() != vector_size || dp_Notch.size() != vector_size )) {
        log(Error)<<"Controller: Could not configure component: Wrong size of one of the notch filter!"<<endlog();
        return false;
    }
    if ( LowPass && (fp_LowPass.size() != vector_size || dp_LowPass.size() != vector_size )) {
        log(Error)<<"Controller: Could not configure component: Wrong size of one of the low pass!"<<endlog();
        return false;
    }

    // Check input data signs
    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"Controller: Could not configure component: vector_size or Ts parameter is invalid!"<<endlog();
        return false;
    }
    for (uint i = 0; i < vector_size; i++) {
        if (WeakIntegrator) {
            if ( fz_WeakIntegrator[i] < 0.0) {
                log(Error)<<"Controller: Could not configure component: Wrong sign of one of the weak integrator parameters!"<<endlog();
                return false;
            }
        }
        if (LeadLag) {
            if ( fz_LeadLag[i] < 0.0 || fp_LeadLag[i] < 0.0 ) {
                log(Error)<<"Controller: Could not configure component: Wrong sign of one of the leadlag parameters!"<<endlog();
                return false;
            }
        }
        if (Notch) {
            if ( fz_Notch[i] < 0.0 || dz_Notch[i] < 0.0 || fp_Notch[i] < 0.0 || dp_Notch[i] < 0.0) {
                log(Error)<<"Controller: Could not configure component: Wrong sign of one of the notch parameters!"<<endlog();
                return false;
            }
        }
        if (LowPass) {
            if ( fp_LowPass[i] < 0.0 || dp_LowPass[i] < 0.0 ) {
                log(Error)<<"Controller: Could not configure component: Wrong sign of one of the low pass parameters!"<<endlog();
                return false;
            }
        }
    }
    
    // Check
	if ( (N_refinports < 1) || (N_refinports > 3) ) {
		log(Error)<<"Controller: Could not configure component: The number of reference input ports: " << N_refinports << ", should be at least 1 and at most 3!"<<endlog();
		return false;
	}
	if ( (N_ffwinports < 0) || (N_ffwinports > 3) ) {
		log(Error)<<"Controller: Could not configure component: The number of ffw input ports: " << N_ffwinports << ", Cant be negative and can be at most 3!"<<endlog();
		return false;
	}

    return true;
}

bool Controller::startHook()
{
    // Check validity of Ports:
    if ( !positions_inport.connected() ) {
        log(Error)<<"Controller: Could not start component: pos_in is not connected!"<<endlog();
        return false;
    }
    
    if ( !controleffort_outport.connected() ) {
        log(Error)<<"Controller: Could not start component:  Outputport not connected!"<<endlog();
        return false;
    }    
    for (uint j = 0; j < N_ffwinports; j++) {
        if ( !ffw_inport[j].connected() ) {
            log(Error)<<"Controller: Could not start component: ffw_in" << j + 1 << " not connected!"<<endlog();
			return false;
		}
	}
    for (uint j = 0; j < N_refinports; j++) {
        if ( !references_inport[j].connected() ) {
            log(Error)<<"Controller: Could not start component: ref_in" << j + 1 << " not connected!"<<endlog();
            return false;
        }
    }

    // Initialize
    safe = false;

    return true;
}

void Controller::updateHook()
{
    // Read positions
    doubles positions(vector_size,0.0);
    positions_inport.read(positions);

    // Read all reference ports
    doubles references(vector_size,0.0);
    uint k = 0;
    for ( uint j = 0; j < N_refinports; j++ ) {
        doubles ref_in(inport_sizes[j],0.0);
        references_inport[j].read(ref_in);
        for ( uint i = 0; i < inport_sizes[j]; i++ ) {
            references[k+i] = ref_in[i];
            k++;
        }
    }

    // Read (possible) ffw ports
    vector<doubles> ffw_input;
    ffw_input.resize(N_ffwinports);
    for (uint j = 0; j < N_ffwinports; j++) {
        ffw_input[j].assign(vector_size,0.0);
        ffw_inport[j].read(ffw_input[j]);
    }

    // Read safety boolean
    enable_inport.read(safe);

    if (!safe) {
        // Write zero outputs
        doubles zero_output(vector_size,0.0);
        jointerrors_outport.write(zero_output);
        controleffort_outport.write(zero_output);

        return ;

    } else {

        doubles jointErrors(vector_size,0.0);
        doubles output_Gains(vector_size,0.0);
        doubles output_WeakIntegrator(vector_size,0.0);
        doubles output_LeadLag(vector_size,0.0);
        doubles output_Notch(vector_size,0.0);
        doubles output(vector_size,0.0);

        for (uint i = 0; i < vector_size; i++) {
			// Compute joint errors and Apply Gain
            jointErrors[i] = references[i]-positions[i];
            output_Gains[i] = jointErrors[i]*gains[i];
        
			// Apply Weak Integrator
			if (WeakIntegrator) {
				filters_WeakIntegrator[i]->update( output_Gains[i] );
				output_WeakIntegrator[i] = filters_WeakIntegrator[i]->getOutput();
			} else {
				output_WeakIntegrator[i] = output_Gains[i];
			}
			
			// Apply Lead Lag
			if (LeadLag) {
				filters_LeadLag[i]->update( output_WeakIntegrator[i] );
				output_LeadLag[i] = filters_LeadLag[i]->getOutput();
			} else {
				output_LeadLag[i] = output_WeakIntegrator[i];
			}
			
			// Apply Notch
			if (Notch) {
				filters_Notch[i]->update( output_LeadLag[i] );
				output_Notch[i] = filters_Notch[i]->getOutput();
			} else {
				output_Notch[i] = output_LeadLag[i];
			}
			
			// Apply Low Pass
			if (LowPass) {
				filters_LowPass[i]->update( output_Notch[i] );
				output[i] = filters_LowPass[i]->getOutput();
			} else {
				output[i] = output_Notch[i];
			}

			// Add possible feed forward inputs to the controller 
            for (uint j = 0; j < N_ffwinports; j++) {
                output[i] += ffw_input[j][i];
			}
        }

        // Write the outputs
        jointerrors_outport.write(jointErrors);
        controleffort_outport.write(output);
    }
}


void Controller::stopHook()
{
    // Write zero outputs
    doubles zero_output(vector_size,0.0);
    jointerrors_outport.write(zero_output);
    controleffort_outport.write(zero_output);

    // reset filters
    if (WeakIntegrator) {
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_WeakIntegrator[i]->finalize();
            filters_WeakIntegrator[i]->configure(fz_WeakIntegrator[i], Ts, 4);
        }
    }
    if (LeadLag) {
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_LeadLag[i]->finalize();
            filters_LeadLag[i]->configure(fz_LeadLag[i], fp_LeadLag[i], Ts, 4);
        }
    }
    if (Notch) {
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_Notch[i]->finalize();
            filters_Notch[i]->configure(fz_Notch[i], dz_Notch[i], fp_Notch[i], dp_Notch[i], Ts, 4);
        }
    }
    if (LowPass) {
        for (uint i = 0; i < vector_size; i++) {
            // Default discretization method: Prewarp Tustin
            filters_LowPass[i]->finalize();
            filters_LowPass[i]->configure(fp_LowPass[i], dp_LowPass[i], Ts);
        }
    }

    return ;
}

ORO_CREATE_COMPONENT(FILTERS::Controller)
