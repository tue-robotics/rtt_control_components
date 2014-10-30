/** Abs.hpp
 *
 * @class Abs
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef ABS_HPP
#define ABS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.

using namespace std;

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
   * @brief A Component that calculates absolute value of the 
   *        input signals
   *
   * The component has input port as stated in property number_of_inputs,
   * that should receive vector of doubles.
   * Input ports are eventports which will trigger the component.
   * 
   * @param * N - number of inputs
   *        * vector_size - size of input vectors
   */
   
  class Abs
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

		Abs(const string& name);
		~Abs();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
