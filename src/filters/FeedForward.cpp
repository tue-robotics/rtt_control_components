/** FeedForward.cpp
 *
 * @class FeedForward
 *
 * \author Ton Peters
 * \date September, 2014
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "FeedForward.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace FILTERS;

FeedForward::FeedForward(const string& name) :
    TaskContext(name, PreOperational)
{
    // Properties
    addProperty("vector_size",                   vector_size)           .doc("Number of feed forward components");
    
    // Gain arrays, fill in only for the used feed forward.
    addProperty("coulomb_gain",                  coulomb_gain)          .doc("Coulomb gains");
    addProperty("viscous_gain",                  viscous_gain)          .doc("Viscous gains");
    addProperty("acceleration_gain",             acceleration_gain)     .doc("Acceleration gains");
    addProperty("direction_gain",                direction_gain)        .doc("Direction gains");

    // Adding ports
    addPort( "vel_in",                          inport_velocity)        .doc("Velocity reference port");
    addPort( "acc_in",                          inport_acceleration)    .doc("Acceleration reference port");
    addPort( "out",                             outport_feedforward)    .doc("Feed Forward output port");
}

FeedForward::~FeedForward(){}

bool FeedForward::configureHook()
{
	Logger::In in("FeedForward::Configure");	
	
    if (vector_size < 1) {
        log(Error) << "FeedForward:: wrong vector_size specified" << endlog();
        return false;
    }

    // set the default values to zero
    if ( coulomb_gain.size() != vector_size ){
        coulomb_gain.assign(vector_size,0.0);
    }
    if ( viscous_gain.size() != vector_size ){
        viscous_gain.assign(vector_size,0.0);
    }
    if ( acceleration_gain.size() != vector_size ){
        acceleration_gain.assign(vector_size,0.0);
    }
    if ( direction_gain.size() != vector_size ){
        direction_gain.assign(vector_size,0.0);
    }

    // define zero output
    zero_output.assign(vector_size,0.0);
	return true;
}

bool FeedForward::startHook()
{
	Logger::In in("FeedForward::Start");	
	
    // check validity of properties
    if ( coulomb_gain.size() != vector_size || viscous_gain.size() != vector_size || acceleration_gain.size() != vector_size || direction_gain.size() != vector_size){
        log(Error) << "FeedForward:: One of the gain properties is not set correctly" << endlog();
        return false;
    }
    
    // check if ports are connected
    if ( !inport_velocity.connected() ){
        log(Warning) << "FeedForward:: velocity inport (vel_in) not connected" << endlog();
    }

    return true;
}

void FeedForward::updateHook()
{
	Logger::In in("FeedForward::Update");	
	
    doubles velocities(vector_size,0.0);
    doubles accelerations(vector_size,0.0);
    doubles output(vector_size,0.0);

    // Read the input ports
    if ( inport_velocity.connected() ){
        inport_velocity.read(velocities);
    }
    if ( inport_acceleration.connected() ){
        inport_acceleration.read(accelerations);
    }

    // Calculate feed forward effort
    for ( uint i = 0; i < vector_size; i++ ){
        double coulomb = 0.0;
        if ( velocities[i] > 0.0 ){
            coulomb = coulomb_gain[i] + direction_gain[i];
        } else if ( velocities[i] < 0.0 ) {
            coulomb = -coulomb_gain[i] + direction_gain[i];
        }
        output[i] = coulomb + velocities[i] * viscous_gain[i] + accelerations[i] * acceleration_gain[i];
    }
   
    // Write the outputs
    outport_feedforward.write(output);
    
}

void FeedForward::stopHook()
{
	outport_feedforward.write(zero_output);
}

ORO_CREATE_COMPONENT(FILTERS::FeedForward)
