#ifndef HOMING_HPP
#define HOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace SUPERVISORY
{
    /*! \class Homing
    *  	\brief Defines Orocos component for homing hardware
    *
    *   Component should run on Ts
    *
    *   prestate 0 - send homing reference
    *   state 1 - evaluate homing criterion     true    -> send joint to midpos and go to state 2
    *                                           false   -> wait for true
    *   state 2 - evaluate mid pos reached      true    -> go to next joint and prestate 0
    *                                           false   -> wait for true
    *
    *   functions
    *   evaluateHomingCriterion(jointNr)
    */


    typedef vector<double> doubles;
    typedef vector<int> ints;

    class Homing
    : public RTT::TaskContext
    {
        private:
        // inports
        InputPort<doubles> pos_inport;
        InputPort<std_msgs::Bool> endswitch_inport;
        InputPort<doubles> jointerrors_inport;
        InputPort<doubles> absPos_inport;
        InputPort<doubles> forces_inport;

        // outports
        OutputPort<doubles> ref_outport[5];
        // OutputPort<doubles> homing_logport;
        OutputPort<bool> homingfinished_outport;

        // Properties
        uint N;
        uint N_outports;
        ints outport_sizes;
        uint cntr;
        string bodypart;
        string prefix;

        ints homing_type;
        ints require_homing;
        ints homing_order;
        ints homing_direction;
        doubles homing_velocity;
        doubles homing_stroke;
        doubles reset_stroke;
        doubles homing_endpos;

        doubles homing_forces;
        doubles homing_errors;
        ints homing_absPos;

        // Constants
        bool endswitchhoming;
        bool absolutehoming;
        bool forcehoming;
        bool errorhoming;
        doubles initial_maxerr;
        doubles initial_minpos;
        doubles initial_maxpos;
        doubles initial_maxvel;

        // variables
        int jointNr;
        int state;
        bool joint_finished;
        bool finished;
        double homing_stroke_goal;
        doubles position;
        doubles ref_out;
        doubles ref_out_prev;
        doubles updated_maxerr;
        doubles updated_minpos;
        doubles updated_maxpos;
        doubles updated_maxvel;

        protected:
        TaskContext* Supervisor;
        TaskContext* ReadEncoders;
        TaskContext* Safety;
        TaskContext* ReferenceGenerator;
        Attribute<doubles> Safety_maxJointErrors;
        Attribute<doubles> ReferenceGenerator_minpos;
        Attribute<doubles> ReferenceGenerator_maxpos;
        Attribute<doubles> ReferenceGenerator_maxvel;
        OperationCaller<bool(string)> StartBodyPart;
        OperationCaller<bool(string)> StopBodyPart;
        OperationCaller<void(int,double)> ResetEncoder;
        OperationCaller<void()> ResetReference;

        public:

        Homing(const string& name);
        ~Homing();

        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        
        bool evaluateHomingCriterion(uint jointID);
        void updateHomingRef(uint jointID);
        void sendRef(doubles output_total);


    };
}
#endif
