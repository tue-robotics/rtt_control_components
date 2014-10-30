/** Demux.hpp
 *
 * @class Demux
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef DEMUX_HPP
#define DEMUX_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <sstream>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.

using namespace std;

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
   * @brief A Component that demuxes the input vector signal, i.e. 
   *        breaks it into n-times scalar output
   *
   * The component has one input port that should receive vector of 
   * doubles. The output ports send out a scalar value. Number of 
   * output ports corresponds to the length of the input vector.
   *
   * @param N [2] - number of output ports/length of the input vector
   */
   
  class Demux
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inport;
		OutputPort<double> outports[maxN];

		/* Declaring variables set by properties */
		// Number of output ports
		uint N;
		
    public:

		Demux(const string& name);
		~Demux();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
