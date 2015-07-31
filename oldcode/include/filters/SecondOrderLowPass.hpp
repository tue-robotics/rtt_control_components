/** SecondOrderLowPasses.hpp
 *
 * @class SecondOrderLowPasses
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#ifndef SECONDORDERLOWPASSES_HPP
#define SECONDORDERLOWPASSES_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DSecondOrderLowpass.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{

	typedef vector<double> doubles;

	/**
	   * @brief A Component that acts as a 2nd order low-pass filter
	   *
	   * The component has one input port that should receive scalar.
	   *
	   * @param * fp [100 Hz] - pole frequency of the filter
	   *        * dp [0.1] - pole damping
	   */

	class SecondOrderLowPasses
			: public RTT::TaskContext
	{
	private:

		/* Declaring input and output ports*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		// Vector of pointers to filters
		vector<DFILTERS::DSecondOrderLowpass*> filters;

		/* Declaring variables set by properties */
		// Filter parameters
		doubles fp;
		doubles dp;
		uint vector_size;
		double Ts;

	public:

		SecondOrderLowPasses(const string& name);
		~SecondOrderLowPasses();

		bool configureHook();
		bool startHook();
		void updateHook();

	};
}
#endif
