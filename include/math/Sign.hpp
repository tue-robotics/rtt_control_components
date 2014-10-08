/** Sign.hpp
 *
 * @class Sign
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef SIGN_HPP
#define SIGN_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created

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
	typedef vector<double> doubles;
	
  /**
   * @brief A Component that sets value 1 on output for positive inputs,
   *        value -1 for negative inputs, and value 0 for 0 inputs
   *
   * The component has input port as stated in property number_of_inputs,
   * that should receive vector of doubles.
   * Input ports are eventports which will trigger the component.
   *
   * @param * N - number of inputs
   *        * vector_size - size of input vectors
   */

  class Sign
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

		Sign(const string& name);
		~Sign();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
