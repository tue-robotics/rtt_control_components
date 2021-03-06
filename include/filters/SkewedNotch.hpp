/** SkewedNotch.hpp
 *
 * @class SkewedNotch
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#ifndef SKEWEDNOTCH_HPP
#define SKEWEDNOTCH_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DSkewedNotch.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{
	typedef vector<double> doubles;

	/**
	   * @brief A Component that acts as a skewed-notch filter
	   *
	   * The component has one input port that should receive vector of doubles.
	   *
	   * @param * fz [Hz] - zero frequency of the filter
	   *        * dz [-] - zero damping
	   *        * fp [Hz] - pole frequency of the filter
	   *        * dp [-] - pole damping
	   *        * vector_size [0] - size of input vector
	   *        * Ts [0.0 sec] - sampling time
	   */

	class SkewedNotch
			: public RTT::TaskContext
	{
	private:

		/* Declaring input and output ports*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		// Vector of pointers to filters
		vector<DFILTERS::DSkewedNotch*> filters;

		/* Declaring variables set by properties */
		// Filter parameters
		doubles fz;
		doubles dz;
		doubles fp;
		doubles dp;
		uint vector_size;
		double Ts;

	public:

		SkewedNotch(const string& name);
		~SkewedNotch();

		bool configureHook();
		bool startHook();
		void updateHook();

	};
}
#endif
