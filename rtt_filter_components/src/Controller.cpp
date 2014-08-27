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
    addProperty("zero_freq_WeakIntegrator",     fz_WeakIntegrator)      .doc("zero frequency of the weak integrator");
    addProperty("zero_freq_LeadLag",            fz_LeadLag)             .doc("zero frequency of the lead lag filter");
    addProperty("pole_freq_LeadLag",            fp_LeadLag)             .doc("pole frequency of the lead lag filter");
    addProperty("zero_frequency_Notch",         fz_Notch)               .doc("zero frequency of the Notch filter");
    addProperty("zero_damping_Notch",           dz_Notch)               .doc("zero damping of the Notch filter");
    addProperty("pole_frequency_Notch",         fp_Notch)               .doc("pole frequency of the Notch filter");
    addProperty("pole_damping_Notch",           dp_Notch)               .doc("pole damping of the Notch filter");
    addProperty("pole_freq_LowPass",            fp_LowPass)             .doc("pole frequency of the low pass filter");
    addProperty("pole_damp_LowPass",            dp_LowPass)             .doc("pole frequency of the lead lag filter");
    addProperty("sampling_time",                Ts)                     .doc("Sampling time");
    addProperty("max_errors",                   max_errors)             .doc("Maximum allowed joint errors");
    addProperty("motor_saturation",             motor_saturation)       .doc("Motor saturation level");
    addProperty("max_sat_time",                 max_sat_time)           .doc("Maximum time the motors are allowed to exceed the motor_saturation value");

    // Adding ports
    addPort( "ref_in",                          inport_references)      .doc("Control Reference port");
    addEventPort( "pos_in",                     inport_positions)       .doc("Position Port");
    addPort( "out",                             outport_controloutput)  .doc("Control output port");
    addPort( "safe",                            outport_safety)         .doc("Boolean Safety Port, safe=true means safe and safe=false disables all actuators");
    addPort( "controlerrors",                   outport_controlerrors)  .doc("Controlerrors output port % publishes at 10Hz");
}

Controller::~Controller(){}

bool Controller::configureHook()
{
    firstSatInstance.assign(vector_size,0);
    firstErrInstance.assign(vector_size,0);
    timeReachedSaturation.assign(vector_size,0.0);
    zero_output.assign(vector_size,0.0);

    filters_WeakIntegrator.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {
        // Default discretization method: Prewarp Tustin
        filters_WeakIntegrator[i] = new DFILTERS::DWeakIntegrator(fz_WeakIntegrator[i], Ts, 4);
    }

    filters_LeadLag.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {
        // Default discretization method: Prewarp Tustin
        filters_LeadLag[i] = new DFILTERS::DLeadLag(fz_LeadLag[i], fp_LeadLag[i], Ts, 4);
    }

    filters_Notch.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {
        // Default discretization method: Prewarp Tustin
        filters_Notch[i] = new DFILTERS::DSkewedNotch(fz_Notch[i], dz_Notch[i], fp_Notch[i], dp_Notch[i], Ts, 4);
    }

    filters_LowPass.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {
        // Default discretization method: Prewarp Tustin
        filters_LowPass[i] = new DFILTERS::DSecondOrderLowpass(fp_LowPass[i], dp_LowPass[i], Ts);
    }

    return true;
}

bool Controller::startHook()
{
    errors=false;
    cntr = 0;
    cntr_10hz = (int) 1.0/(Ts*10.0); // determine value to which cntr has to go to publish at 10hz

    // Check validity of Ports:
    if ( !inport_references.connected() || !inport_positions.connected() ) {
        log(Error)<<"Controller: One of the inports is not connected!"<<endlog();
        return false;
    }

    if ( !outport_controloutput.connected() ) {
        log(Error)<<"Controller: Outputport not connected!"<<endlog();
     //   return false;
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"Controller:: vector_size or Ts parameter is invalid!"<<endlog();
        return false;
    }
    
    if (fz_WeakIntegrator.size() != vector_size || fz_LeadLag.size() != vector_size || fp_LeadLag.size() != vector_size || fp_LowPass.size() != vector_size || dp_LowPass.size() != vector_size || gains.size() != vector_size || max_errors.size() != vector_size || motor_saturation.size() != vector_size ) {
        log(Error)<<"Controller:: The size of one of your vector inputs is not equal to the vector_size parameter!"<<endlog();    
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fz_WeakIntegrator[i] < 0.0 || fz_LeadLag[i] < 0.0 || fp_LeadLag[i] < 0.0 || fp_LowPass[i] < 0.0 || dp_LowPass[i] < 0.0 ) {
            log(Error)<<"Controller:: One of the zero's, poles or damping coefficients of your filters is below zero!"<<endlog();
            return false;
        }
    }

    return true;
}

void Controller::updateHook()
{
    doubles references(vector_size,0.0);
    doubles positions(vector_size,0.0);
    doubles controlerrors(vector_size,0.0);
    doubles output_G(vector_size,0.0);
    doubles output_WeakIntegrator(vector_size,0.0);
    doubles output_LeadLag(vector_size,0.0);
    doubles output_Notch(vector_size,0.0);
    doubles output(vector_size,0.0);

    // Read the input ports
    inport_references.read(references);
    inport_positions.read(positions);

    // Compute joint errors
    for (uint i = 0; i < vector_size; i++) {
        controlerrors[i] = references[i]-positions[i];
    }

    // Apply controller 1 - Gain
    for (uint i = 0; i < vector_size; i++) {
        output_G[i] = controlerrors[i]*gains[i];
    }

    // Apply controller 2 - Weak Integrator
    for (uint i = 0; i < vector_size; i++) {
        filters_WeakIntegrator[i]->update( output_G[i] );
        output_WeakIntegrator[i] = filters_WeakIntegrator[i]->getOutput();
    }

    // Apply controller 3 - Lead Lag
    for (uint i = 0; i < vector_size; i++) {
        filters_LeadLag[i]->update( output_WeakIntegrator[i] );
        output_LeadLag[i] = filters_LeadLag[i]->getOutput();
    }

    // Apply controller 4 - Notch
    for (uint i = 0; i < vector_size; i++) {
        filters_Notch[i]->update( output_LeadLag[i] );
        output_Notch[i] = filters_Notch[i]->getOutput();
    }

    // Apply controller 5 - Low Pass
    for (uint i = 0; i < vector_size; i++) {
        filters_LowPass[i]->update( output_Notch[i] );
        output[i] = filters_LowPass[i]->getOutput();
    }

    // Safety check 1 - Maximum joint error // to do check if nulling is a problem
    for ( uint i = 0; i < vector_size; i++ ) {
        if( (fabs(controlerrors[i])>max_errors[i] && !errors) ) {
            firstErrInstance[i]++;
            if( errors == false && firstErrInstance[i] == 5){
                ROS_ERROR_STREAM( "Controller: Error of joint q"<<i+1<<" exceeded limit ("<<max_errors[i]<<"). output disabled." );
                errors = true;
            }
        } else {
			firstErrInstance[i] = 0;
        }
    }

    // Safety check 2 - Motor saturation
    long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9;
    for(unsigned int i = 0;i<vector_size;i++){
        if(firstSatInstance[i]==0 && fabs(output[i])>=motor_saturation[i]){
            timeReachedSaturation[i]=timeNow;
            firstSatInstance[i]=1;
        }
        else if(fabs(output[i])<motor_saturation[i]){
            timeReachedSaturation[i]=timeNow;
            firstSatInstance[i]=0;
        }
        if(fabs(timeNow-timeReachedSaturation[i])>=max_sat_time){
            if(errors==false){
                ROS_ERROR_STREAM( "Controller: Motor output "<<i+1<<" satured too long (absolute "<<max_sat_time<<" sec above "<<fabs(motor_saturation[i])<<"). output disabled." );
                errors = true;
            }
        }
    }

    // Write the outputs
    if (!errors) {
        outport_safety.write(true);
        outport_controloutput.write( output );
    }
    else {
        outport_safety.write(false);
        outport_controloutput.write( zero_output );
    }

    if (cntr == cntr_10hz){
        cntr = 0;
        outport_controlerrors.write( controlerrors );
    }
    cntr++;
}

ORO_CREATE_COMPONENT(FILTERS::Controller)