#ifndef GLOBALREFERENCEGENERATOR_HPP
#define GLOBALREFERENCEGENERATOR_HPP

#define maxN 5 //Maximum matrix size

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

	class GlobalReferenceGenerator
		: public RTT::TaskContext
		{
		private:

            // iterators
            // j iterates over all connected bodyparts
            // i iterates over all joints within particular bodypart

			// Declaring input- and output_ports
            InputPort<sensor_msgs::JointState> inport;
			InputPort<doubles> initialposinport[maxN];
			OutputPort<doubles> posoutport[maxN];
			OutputPort<doubles> veloutport[maxN];
			OutputPort<doubles> accoutport[maxN];

			// Properties
			vector<doubles> minpos;
			vector<doubles> maxpos;
			vector<doubles> maxvel;
            vector<doubles> maxacc;

			// Declaring global variables
            ints vector_sizes;
            bool checked;
            bools allowedBodyparts;
            int totalNumberOfJoints;
            int numberOfBodyparts;
            ints activeBodyparts;
            double start_time;
            doubles InterpolDts;
            doubles InterpolEpses;
            vector< vector< refgen::RefGenerator > > mRefGenerators;
            vector< vector< amigo_msgs::ref_point> > mRefPoints;
            vector< doubles > desiredPos;
            vector< doubles > desiredVel;
            vector< doubles > desiredAcc;
            vector< doubles > pos_out;
            vector< doubles > vel_out;
            vector< doubles > acc_out;
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
