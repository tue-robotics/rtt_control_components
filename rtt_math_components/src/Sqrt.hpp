/** Sqrt.hpp
 *
 * @class Sqrt
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef SQRT_HPP
#define SQRT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace MATH
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
 /**
   * @brief A Component that calculates the square root of the input
   *        signals
   *
   * The component has input port as stated in property number_of_inputs,
   * that should receive vector of doubles.
   * Input ports are eventports which will trigger the component.
   *
   * @param * N - number of inputs
   *        * vector_size - size of input vectors
   */
   
  class Sqrt
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inports[maxN];
		OutputPort<doubles> outports[maxN];

		/* Declaring global variables */
		uint N;
		uint vector_size;
	
    public:

		Sqrt(const string& name);
		~Sqrt();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
