#ifndef READTWISTMSG_HPP
#define READTWISTMSG_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>


using namespace std;
using namespace RTT;

namespace MSG // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  typedef vector<string> strings;

  class ReadTwistMsg
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<geometry_msgs::Twist> inport;
    OutputPort<doubles> outport;


    // Declaring global variables
    double aquisition_time;       // Variable to store the last time a reference has been received
    double receive_interval;
    double old_time;
    double previous_references[3];  // Previous values of the received references
    strings names;                // Set to x,y and phi. Used for logging.
    uint status;                  // 0 = never received reference, 1 = reference flow interuped, 2 = receiving references
    double ref_vel[3];
    double ref_vel_prev[3];


    // variables set by properties
    double max_start_vel;         // Maximum value of the initial velocity in the starthook
    doubles max_acc;              // maximum value of the acceleration
    double max_interval;

    public:

    ReadTwistMsg(const string& name);
    ~ReadTwistMsg();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
