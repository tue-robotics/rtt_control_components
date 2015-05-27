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
    typedef vector<bool> bools;

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
        string bodypart;
        string prefix;

        ints homing_type;
        ints require_homing;
        ints homing_order;
        ints homing_direction;
        doubles homingVel;
        doubles desiredVel;
        doubles desiredAcc;
        doubles homing_stroke;
        doubles reset_stroke;
		double InterpolDt;
		double InterpolEps;
		
        doubles homing_forces;
        doubles homing_errors;
        doubles homing_absPos;

        // Variables
        bool homeswitchhoming;
        bool absolutehoming;
        bool forcehoming;
        bool errorhoming;
        bool joint_finished;
        bool finishing;
        bool finishingdone;
        int jointNr;
        int stateA;
        int stateB;
        int partNr;
        double homing_stroke_goal;
        doubles position;
        doubles desiredPos;
        doubles homing_endpos;
        doubles initial_maxerr;
        doubles updated_maxerr;
        bools allowedBodyparts;
		vector< doubles > outpos;
		vector< doubles > outvel;
		vector< doubles > outacc;
		std::vector<refgen::RefGenerator> mRefGenerators;
		std::vector<amigo_msgs::ref_point> mRefPoints;
		
        protected:
        
        // Component Peers
        TaskContext* Supervisor;
        TaskContext* ReadEncoders;
        TaskContext* Safety;
        TaskContext* GripperControl;
        TaskContext* TrajectoryActionlib;
        
        // Properties in Component Peers that homing component can modify
        Attribute<doubles> Safety_maxJointErrors;
        Attribute<bools> AllowReadReferencesRefGen;

        // Functions in Component Peers that homing component can call
        OperationCaller<bool(string)> StartBodyPart;
        OperationCaller<bool(string)> StopBodyPart;
        OperationCaller<void(uint,double)> ResetEncoder;
        OperationCaller<void(int)> ResetReferenceRefGen;
        OperationCaller<void(int,doubles)> SendToPos;

        public:

        Homing(const string& name);
        ~Homing();

        bool configureHook();
        bool startHook();
        void updateHook();

        // Internal functions
        bool evaluateHomingCriterion(uint jointID);
        void updateHomingRef(uint jointID);
        void SendRef();

    };
}
#endif
