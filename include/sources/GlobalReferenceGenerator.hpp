#ifndef GLOBALREFERENCEGENERATOR_HPP
#define GLOBALREFERENCEGENERATOR_HPP

#define maxN 5 //Maximum bodyparts size

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <amigo_ref_interpolator/interpolator.h>
#include <sensor_msgs/JointState.h>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SOURCES
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
    typedef vector<bool> bools;
    typedef vector<string> strings;
    typedef pair<int, int> BodyJointPair;

    /**
     * @brief Global component that receives joint state messages for
     * all bodyparts. This component sorts them, and generates a
     * reference signal for all joints.
     *
     * The component has one input ports which should be connected with
     * stream to the joint state reference topic of the robot. For each
     * added bodypart an initial position input port is added and three
     * outputports for position, velocity and acceleration. If homable,
     * then also a second input port for homing references is added.
     *
     * Start this component in the robot's main ops file and add bodyparts
     * for each bodypart.
     *
     * Every bodypart can be in different states:
     * 1) Operational - The references from the Jointstate topic from
     *                  ROS are used.
     * 2) Homing      - The references from the homing port are used.
     * 3) Disabled    - The actual measured positions are sent as reference
     *                  since then the error is zero in the disabled mode
     *
     * The supervisor component enables and disables each bodypart.
     * The homing component switches a bodypart to the homing state
     *
     * Example for ops file:
     * GlobalReferenceGenerator.AddBodyPart(BODYNUMBER,JOINT_NAMES)
     * GlobalReferenceGenerator.minPos4 			= array ( -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0 )
     * GlobalReferenceGenerator.maxPos4 			= array ( 0.0,  0.0, 	0.0, 0.0, 	0.0, 0.0, 	0.0 )
     * GlobalReferenceGenerator.maxVel4 			= array ( 0.4,  0.5, 	0.6, 0.7, 	0.8, 0.9, 	1.0 )
     * GlobalReferenceGenerator.maxAcc4             = array ( 1.0,  1.0, 	1.0, 1.0, 	1.0, 1.0, 	1.0 )
     * GlobalReferenceGenerator.interpolatorDt4 	= TS
     * GlobalReferenceGenerator.interpolatorEps4 	= 1.0
     *
     */

    /**
    * TODO
    * 1) add homing input port
    * 2) set function to property acces for homing component
    * 3) add property acces for supervisor to disable bodypart
    * 4) add disabled mode where current pos is directly used
    * 5) add multiple input ports to controller component
    */

	class GlobalReferenceGenerator
		: public RTT::TaskContext
		{
		private:

            // iterators
            // j iterates over all connected bodyparts
            // i iterates over all joints within particular bodypart

			// Declaring input- and output_ports
            InputPort<sensor_msgs::JointState> inport;
            InputPort<doubles> currentpos_inport[maxN];
			OutputPort<doubles> posoutport[maxN];
			OutputPort<doubles> veloutport[maxN];
			OutputPort<doubles> accoutport[maxN];

			// Properties
            vector<doubles> minpos;
            vector<doubles> maxpos;
            vector<doubles> maxvel;
            vector<doubles> maxacc;

            // Global variables - scalar
            bool checked;
            int totalNumberOfJoints;
            int numberOfBodyparts;
            double start_time;

            // Global variables - vector
            bools allowedBodyparts;
            ints activeBodyparts;
            ints vector_sizes;
            doubles InterpolDts;
            doubles InterpolEpses;

            // Global variables - vector of vectors
            vector< doubles > desiredPos;
            vector< doubles > desiredVel;
            vector< doubles > desiredAcc;
            vector< doubles > pos_out;
            vector< doubles > vel_out;
            vector< doubles > acc_out;
            vector< vector< refgen::RefGenerator > > mRefGenerators;
            vector< vector< amigo_msgs::ref_point> > mRefPoints;

            // Global variables - map
            map<string, BodyJointPair> joint_map;

		public:

			GlobalReferenceGenerator(const string& name);
			~GlobalReferenceGenerator();

			bool configureHook();
			bool startHook();
			void updateHook();
            void AddBodyPart(int partNr, strings JointNames);
            void AllowReadReference(int partNr, bool allowed);
            void ResetReference(int partNr);
            bool CheckConnectionsAndProperties();


	};
}
#endif
