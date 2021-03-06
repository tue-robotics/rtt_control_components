/** LeadLag.hpp
 *
 * @class LeadLag
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#ifndef LEADLAG_HPP
#define LEADLAG_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DLeadLag.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{

	typedef vector<double> doubles;

	/**
	* @brief A Component that acts as a lead-lag filters for multiple
	*        inputs
	*
	* The component has one input port that should receive vector of
	* doubles.
	*
	* @param * fz[] [Hz] - array of zero frequencies of the filter
	*        * fp[] [Hz] - array of pole frequencies of the filter
	*        * vector_size - size of the array
	*        * Ts - sampling time
	*/

	class LeadLag
			: public RTT::TaskContext
		{
		private:

			/* Declaring input and output ports*/
			InputPort<doubles> inport;
			OutputPort<doubles> outport;

			/* Declaring global variables */
			// Vector of pointers to filters
			vector<DFILTERS::DLeadLag*> filters;

			/* Declaring variables set by properties */
			// Filter parameters
			doubles fz;
			doubles fp;
			uint vector_size;
			double Ts;

		public:

			LeadLag(const string& name);
			~LeadLag();

			bool configureHook();
			bool startHook();
			void updateHook();
	};
}
#endif
