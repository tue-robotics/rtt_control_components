#ifndef HOMING_HPP
#define HOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace SUPERVISORY // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  typedef vector<int> ints;

  class Homing
  : public RTT::TaskContext
    {
    private:
    // Declaring Inports
    InputPort<std_msgs::Bool> endSwitch_inport;
    InputPort<doubles> absPos_inport;
    InputPort<doubles> force_inport;
    InputPort<doubles> servoError_inport;
    InputPort<doubles> relPos_inport;

    // Declaring Outports
    OutputPort< vector<doubles> > ref_outport;
    OutputPort< uint > status_outport;


    // Declaring system variables
    bool homed;
    bool HomingConstraintMet;
    bool GoToMidPos;
    bool increased_vel;
    bool send_new_reference;
    uint N;
    uint JntNr;

    // homing variables
    bool require_homing;
    string homing_body;
    string homing_compname;
    doubles homing_order;
    doubles homing_type;    
    doubles homing_refPos;
    doubles homing_refVel;
    doubles homing_midpos;
    doubles homing_endpos;
    doubles homing_stroke;

    // local homing variables
    int homing_order_t;
    doubles homing_refPos_t;
    doubles homing_refVel_t;

    //ref
    vector<doubles> ref;

    // Current value variables
    std_msgs::Bool endSwitch;
    doubles absPos;
    doubles servoErrors;
    doubles forces;
    doubles relPos;

    // Homing criterion variables
    doubles homing_absPos;
    doubles homing_force;
    doubles homing_error;

    protected:
    OperationCaller<bool(string)> StartBodyPart;
    OperationCaller<bool(string)> StopBodyPart;
    OperationCaller<void(int,double)> ResetEncoders;

    
    public:

    Homing(const string& name);
    ~Homing();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
