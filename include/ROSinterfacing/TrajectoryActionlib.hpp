#ifndef TrajectoryActionlib_HPP
#define TrajectoryActionlib_HPP

#define maxN 10 //Maximum matrix size

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


    class TrajectoryActionlib
		: public RTT::TaskContext
		{
		private:

			// Declaring input- and output_ports
			InputPort<doubles> posinport[5];
			InputPort<doubles> initialposinport;
			OutputPort<doubles> posoutport;
			OutputPort<doubles> veloutport;
			OutputPort<doubles> accoutport;
			OutputPort<sensor_msgs::JointState> resetrefoutport;

			// Properties
			uint N;
			uint N_inports;
			doubles minpos;
			doubles maxpos;
			doubles maxvel;
			doubles maxacc;
			ints inport_sizes;

			// Declaring global variables
			std::vector<refgen::RefGenerator> mRefGenerators;
			std::vector<amigo_msgs::ref_point> mRefPoints;
			doubles desiredPos;
			doubles desiredVel;
			doubles desiredAcc;
			double InterpolDt, InterpolEps;

		public:

            TrajectoryActionlib(const string& name);
            ~TrajectoryActionlib();

			bool configureHook();
			bool startHook();
			void updateHook();
            void resetReference();

	};
}
#endif
