/** SineWave.hpp
 *
 * @class SineWave
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef SINEWAVE_HPP
#define SINEWAVE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <math.h>

using namespace std;
using namespace RTT;

namespace SOURCES
{
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a sine waves
   *
   * The component has no input ports and N output ports with vector of
   * doubles
   *
   * @param * freq [Hz] - sets the frequency of the sine waves
   *        * amplitude [-] - sets the amplitude of the sine waves
   *        * phase [rad] - sets the phase shift of the sine waves
   *        * bias [-] - sets the bias (amplitude shift) of the sine 
   *          waves
   */
   
  class SineWave
  : public RTT::TaskContext
    {
    private:

		/*** Declaring output port ***/
		OutputPort<doubles> outport;

		/*** Declaring global variables ***/
		unsigned int k;
		double Ts;		
		
		/*** Declaring variables set by properties ***/
		doubles freq;
		doubles amplitude;
		doubles phase;
		doubles bias;
		uint vector_size;
		
    public:

		SineWave(const string& name);
		~SineWave();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
