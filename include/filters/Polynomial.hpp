/** Polynomials.hpp
 *
 * @class Polynomials
 *
 * \author Janno Lunenburg
 * \date December, 2013
 * \version 1.0
 *
 */

#ifndef POLYNOMIALS_COMPONENT_HPP
#define POLYNOMIALS_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/polynomial/Polynomial.hpp>

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
    stringstream ss;
    ss << t;
    return ss.str();
};

namespace FILTERS
{
	typedef vector<double> doubles;

	/**
	   * @brief A Component that acts filters inputs periodically
	   *
	   * The component has one input port that should a vector with doubles.
	   *
	   *ToDo: update
	   * @param * kp [-] - proportional gain
	   *        * kv [-] - derivative gain
	   *        * ki [-] - integral gain
	   *        * kaw [-] - anti-windup gain
	   *        * init [-] - initial value of the integrator
	   *        * limit [-] - output limit (saturation value)
	   *        * Ts [0.0 sec] - sampling time
	   *        * vector_size [0] - size of input vector
	   */

	class Polynomials
			: public RTT::TaskContext
	{
	private:

		/* Declaring input and output ports*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		vector<Polynomial> polynomials;

		/* Declaring variables set by properties */
		// Filter parameters
		uint vector_size; // Number of polynomials/size of inputs and outputs
		vector<unsigned int> orders; // Order of the various polynomials
		doubles input, output;

	public:

		Polynomials(const string& name);
		~Polynomials();

		bool configureHook();
		bool startHook();
		void updateHook();

	};
}
#endif
