#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ReadTwistMsg.hpp"

using namespace std;
using namespace RTT;
using namespace MSG;

ReadTwistMsg::ReadTwistMsg(const string& name) : TaskContext(name, PreOperational)
{
  // Adding Properties:
  addProperty( "max_start_vel", max_start_vel );
  addProperty( "max_acc", max_acc );
  addProperty( "max_interval", max_interval );

  // Creating ports:
  addEventPort( "cmd_vel", inport );
  addPort( "out", outport );
}
ReadTwistMsg::~ReadTwistMsg(){}

bool ReadTwistMsg::configureHook()
{
  // Declare names for logging purpose:
  names.push_back("x");
  names.push_back("y");
  names.push_back("phi");


  if ( max_acc.size() != 3 )
  {
    log(Error)<<"ReadTwistMsg::max_acc not specified!"<<endlog();
    return false;
  }



  return true;
}

bool ReadTwistMsg::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() )
  {
    log(Error)<<"ReadTwistMsg::Inputport not connected!"<<endlog();
    return false;
  }
  if ( !outport.connected() ) {
    log(Warning)<<"ReadTwistMsg::Outputport not connected!"<<endlog();
  }

  //Check if values are zero (small) at start
  geometry_msgs::Twist cmd_veldata;
  inport.read(cmd_veldata);
  if ( cmd_veldata.linear.x > max_start_vel )
  {
    log(Error)<<"Cannot start if the reference velocity has a value of "<<cmd_veldata.linear.x<<". A maximum of "<<max_start_vel<<" is allowed"<<endlog();
    return false;
  }
  if ( cmd_veldata.linear.y > max_start_vel )
  {
    log(Error)<<"Cannot start if the reference velocity has a value of "<<cmd_veldata.linear.y<<". A maximum of "<<max_start_vel<<" is allowed"<<endlog();
    return false;
  }
  if ( cmd_veldata.angular.z > max_start_vel )
  {
    log(Error)<<"Cannot start if the reference velocity has a value of "<<cmd_veldata.angular.z<<". A maximum of "<<max_start_vel<<" is allowed"<<endlog();
    return false;
  }
  aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
  //log(Debug)<<"aquisition_time: "<<aquisition_time<<endlog();
  status = 0;

  for ( uint i = 0; i <= 2; i++ )
  {
    previous_references[i] = 0.0;
    ref_vel_prev[i] = 0.0;
  }

  old_time = os::TimeService::Instance()->getNSecs()*1e-9;

  return true;
}


void ReadTwistMsg::updateHook()
{
  // Determine timestamp:
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  //log(Debug)<<"new_time: "<<new_time<<endlog();
  double dt = new_time - old_time;
  old_time = new_time;

  // Read the inputport
  ////log(Debug)<<"Reading cmd_vel"<<endlog();
  geometry_msgs::Twist cmd_veldata;
  doubles references(3,0.0);
  doubles output(3,0.0);


  // Start fresh data check
  if(NewData == inport.read(cmd_veldata))
  {
    log(Info)<<"New Data received"<<endlog();
    receive_interval = new_time - aquisition_time;
    aquisition_time = new_time;
    //log(Debug)<<"aquisition_time: "<<aquisition_time<<endlog();
    status = 2;
  }

  double receive_delay = new_time - aquisition_time;
  //log(Debug)<<"receive_delay: "<<receive_delay<<endlog();
  if ( receive_delay > max_interval )
  {
    if ( status == 2 )
    {
      log(Warning)<<"Stopped driving because the reference-flow was interrupted."<<endlog();
      status = 1;
    }
    for ( uint i = 0; i <= 2; i++ )
      ref_vel[i] = 0.0;
  }
  else
  {
    references[0] = cmd_veldata.linear.x;
    references[1] = cmd_veldata.linear.y;
    references[2] = cmd_veldata.angular.z;
  }

  // Limit output:
  for ( uint i = 0; i <= 2; i++ )
  {
    double acc = (references[i] - ref_vel_prev[i])/dt;
    acc = max(-max_acc[i], min(max_acc[i], acc));
    ref_vel[i] = ref_vel_prev[i] + acc * dt;
    ref_vel_prev[i] = ref_vel[i];
    //ref_vel[i] = references[i];
  }

  //log(Debug)<<"Writing to port"<<endlog();
  // Write data to port
  for ( uint i = 0; i <= 2; i++ )
    output[i] = ref_vel[i];

  outport.write( output );
}

ORO_CREATE_COMPONENT(MSG::ReadTwistMsg)
