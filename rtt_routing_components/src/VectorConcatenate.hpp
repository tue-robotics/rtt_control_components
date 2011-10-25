/** VectorConcatenate.hpp
 *
 * @class VectorConcatenate
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef VECTORCONCATENATE_HPP
#define VECTORCONCATENATE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <sstream>

using namespace RTT;

namespace SIGNALROUTING
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
    /**
   * @brief A Component that concatenates input vectors into one
   *        output vector
   *
   * The component has N input ports that should receive vectors of the
   * same size. It outputs one vector of the length that equals the 
   * sum of the lengths of input vectors
   *
   * @param N [2] - number of input ports (vectors)
   */

  class VectorConcatenate
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		vector< InputPort<doubles> > inports;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		double Ts;		
		
		/* Declaring variables set by properties */
		// Number of input ports
		uint N;
		
    public:

		VectorConcatenate(const string& name);
		~VectorConcatenate();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
