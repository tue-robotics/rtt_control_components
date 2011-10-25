/** Mux.hpp
 *
 * @class Mux
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef MUX_HPP
#define MUX_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <sstream>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SIGNALROUTING
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
    /**
   * @brief A Component that muxes the input signals, i.e. 
   *        puts together all the inputs into one vector output
   *
   * The component has N input ports that should receive scalar of type
   * double. The output port send out a vector. Length of the output
   * vector corresponds to the number of input ports
   *
   * @param N [2] - number of input ports/length of the output vector
   */

  class Mux
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<double> inports[maxN];
		OutputPort<doubles> outport;

		/* Declaring variables set by properties */
		// Number of input ports
		uint N;
		
    public:

		Mux(const string& name);
		~Mux();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
