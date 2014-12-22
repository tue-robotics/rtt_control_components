#ifndef HOMING_HPP
#define HOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <amigo_ref_interpolator/interpolator.h>

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
        // Ports
        InputPort<doubles> pos_inport;
        InputPort<std_msgs::Bool> homeswitch_inport;
        InputPort<doubles> jointerrors_inport;
        InputPort<doubles> absPos_inport;
        InputPort<doubles> forces_inport;
        OutputPort<doubles> posoutport[2];
        OutputPort<doubles> veloutport[2];
        OutputPort<doubles> accoutport[2];
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
        doubles desiredVel;
        doubles desiredAcc;
        doubles homing_stroke;
        doubles reset_stroke;
        doubles homing_endpos;

        doubles homing_forces;
        doubles homing_errors;
        ints homing_absPos;

        // Constants
        bool homeswitchhoming;
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
        bool finishing;
        double homing_stroke_goal;
        doubles position;
        doubles desiredPos;
        doubles updated_maxerr;
        doubles updated_minpos;
        doubles updated_maxpos;
        doubles updated_maxvel;
		std::vector<refgen::RefGenerator> mRefGenerators;
		std::vector<amigo_msgs::ref_point> mRefPoints;
		double InterpolDt;
		double InterpolEps;
		vector< doubles > outpos;
		vector< doubles > outvel;
		vector< doubles > outacc;

        protected:
        // Component Peers
        TaskContext* Supervisor;
        TaskContext* ReadEncoders;
        TaskContext* Safety;
        // Properties in Component Peers that homing component can modify
        Attribute<doubles> Safety_maxJointErrors;

        // Functions in Component Peers that homing component can call
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

        // Internal functions
        bool evaluateHomingCriterion(uint jointID);
        void updateHomingRef(uint jointID);

    };
}
#endif
