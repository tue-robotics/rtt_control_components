/** BenchmarkReference.hpp
 *
 * @class BenchmarkReference
 *
 * \author Max Baeten
 * \date April, 2014
 * \version 1.0
 *
 */

#ifndef BENCHMARKREFERENCE_HPP
#define BENCHMARKREFERENCE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

#define maxN 10

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace SOURCES
{
  class BenchmarkReference
  : public RTT::TaskContext
    {
		typedef vector<double> doubles;
		private:
			// Ports
			OutputPort<doubles> position_outport;
			
			//Variables
			doubles referenceFunction[maxN];
			doubles timeFunction;
			doubles pos_out;
			uint N;
			uint cntr_ms;
			uint cntr_s;
			uint timeFunction_j;
			bool endOfReferenceReached;

		public:
			BenchmarkReference(const string& name);
			~BenchmarkReference();

			bool configureHook();
			bool startHook();
			void updateHook();

    };
}
#endif
