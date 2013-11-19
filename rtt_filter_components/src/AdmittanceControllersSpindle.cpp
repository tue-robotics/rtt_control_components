/** AdmittanceControllersSpindle.cpp
*
* @class AdmittanceControllersSpindle
*
* \author Janno Lunenburg
* \date August, 2013
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AdmittanceControllersSpindle.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

AdmittanceControllersSpindle::AdmittanceControllersSpindle(const string& name) :
    TaskContext(name, PreOperational),
    vector_size(0), Ts(0.0)
{

    addProperty( "vector_size", vector_size );
    addProperty( "sampling_time", Ts );
    addProperty( "masses", M );
    addProperty( "damping_coefficients", D );
    addProperty( "lower_joint_limits", lower_bound );
    addProperty( "upper_joint_limits", upper_bound );
    addPort( "homing_finished", homingfinished_inport );

    // Adding ports
    addEventPort( "measured_position_in", position_inport );
    addPort( "force_in", force_inport );
    addPort( "out", outport );
}

AdmittanceControllersSpindle::~AdmittanceControllersSpindle()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters[i];
    }
}

bool AdmittanceControllersSpindle::configureHook()
{
	finishedhoming = false;	
    // Compute fp and dp for first order lowpass filter
    // and Construct filters
    fp.resize(vector_size);
    k.resize(vector_size);
    filters.resize(vector_size);
    for (unsigned int i = 0; i < vector_size; i++)
    {
        k[i] = 1/D[i];
        double op = D[i]/M[i]; //Pole frequency in rad/s
        fp[i] = op/(2*PI);
        filters[i] = new DFILTERS::DFirstOrderLowpass(fp[i], Ts);
    }

    force_input.assign(vector_size, 0.0);
    position_input.assign(vector_size, 0.0);
    position_output.assign(vector_size, 0.0);
    previous_position_output.assign(vector_size, 0.38); // Beuned to the Max: startvalue of spindlecontroller added.
    
    //ToDo: check whether input is valid, e.g., whether all vector lengths coincide 

    return true;
}

bool AdmittanceControllersSpindle::startHook()
{
    // Check validity of Ports:
    if ( !position_inport.connected() ) {
        log(Error)<<"AdmittanceControllersSpindle::position_inputport not connected!"<<endlog();
        // No connection was made, can't do my job !
        return false;
    }

    if ( !force_inport.connected() ) {
        log(Error)<<"AdmittanceControllersSpindle::force_inputport not connected!"<<endlog();
        // No connection was made, can't do my job !
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"AdmittanceControllersSpindle::Outputport not connected!"<<endlog();
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"AdmittanceControllersSpindle parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fp[i] <= 0.0 || k[i] <= 0.0){
            log(Error)<<"AdmittanceControllersSpindle parameters not valid!"<<endlog();
            return false;
        }
    }

    // Read ports to initialize
    force_inport.read(force_input);
    position_inport.read(position_input);
    position_inport.read(previous_position_output);
    return true;
}

void AdmittanceControllersSpindle::updateHook()
{
    // Read inports
    position_inport.read(position_input);
    if (force_inport.read(force_input) == NewData)
    {
		//log(Warning)<<"Admittance: received new data"<<endlog();
        timestamp_last_force_input = os::TimeService::Instance()->getNSecs()*1e-9;
        //log(Warning)<<"Admittance: stamped time"<<endlog();
    }

    // Safety check
    long double current_timestamp = os::TimeService::Instance()->getNSecs()*1e-9;
    double duration = (double)(current_timestamp - timestamp_last_force_input);
    bool force_input_safe = true;
    for (unsigned int i = 0; i < vector_size; i++)
    {
        // If: ( do_it_only_once && force is unequal to zero && time since last update exceeds threshold )
        if ( force_input_safe && fabs(force_input[i]) > eps && duration > 0.11)
        {
            force_input_safe = false;
            //log(Warning) << "No new force input received for " << duration << "seconds, setting force to zero" << endlog();
            force_input.assign(vector_size, 0.0);
        }
    }

    // Update stuff
    for (unsigned int i = 0; i < vector_size; i++)
    {
        // ToDo: is it more efficient to do this (less readable) in less lines?

        // Update
        filters[i]->update(force_input[i]);
        double filter_out = filters[i]->getOutput();
        double v_desired = k[i] * filter_out;

        // Integrate
        //position_output[i] = position_input[i] + v_desired * Ts;
        position_output[i] = previous_position_output[i] + v_desired * Ts;

        // Saturate
        position_output[i] = max(position_output[i], lower_bound[i]);
        position_output[i] = min(position_output[i], upper_bound[i]);
        
        previous_position_output[i] = position_output[i];
    }

    // Write results
    //log(Warning)<<"r = "<<position_output[0]<<" "<<position_output[1]<<" "<<position_output[2]<<" "<<position_output[3]<<" "<<position_output[4]<<" "<<position_output[5]<<" "<<position_output[6]<<endlog();
    
    
    if (NewData == homingfinished_inport.read( finishedhoming ) ){	  
	}
  
  if (finishedhoming) {
	  outport.write(position_output);
  } 
    
}

ORO_CREATE_COMPONENT(FILTERS::AdmittanceControllersSpindle)
