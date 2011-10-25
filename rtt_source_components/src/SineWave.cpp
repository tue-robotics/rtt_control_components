/** SineWave.cpp
*
* @class SineWave
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/
 
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "SineWave.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace RTT;
using namespace SOURCES;

SineWave::SineWave(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(1)
{
  /*** Reading properties ***/
  addProperty( "frequency", freq ).doc("A double value that specifies the frequency of the sine wave in Hz.");
  addProperty( "amplitude", amplitude ).doc("A double value that specifies the amplitude of the sine wave.");
  addProperty( "phase", phase ).doc("A double value that specifies the phase of the sine wave in radians.");
  addProperty( "bias", bias ).doc("A double value that specifies the bias of the sine wave.");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the output vector.");
}

SineWave::~SineWave(){}

bool SineWave::configureHook()
{
  Logger::In in("SineWave::configureHook()");
  
  /*** Adding ports ***/
  addPort( "out", outport ).doc("Vector of double values");
  
  Ts = getPeriod();
  
  if(bias.size() == 0) {
	  bias.resize(vector_size);
  }
  
  if(phase.size() == 0) {
	  bias.resize(vector_size);
  }
  
  if(amplitude.size() == 0) {
	  amplitude.resize(vector_size);
	  for ( uint i = 0; i < vector_size; i++ ) {
		  amplitude[i]=1.0;
	  }
  }
  
  return true;
}

bool SineWave::startHook()
{
  Logger::In in("SineWave::startHook()");
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }
  
  if (vector_size < 1) {
    log(Error)<<"ConstantSignal parameters not valid!"<<endlog();
    return false;
  }

  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }
  
  for ( uint i = 0; i < vector_size; i++ ) {
	  if (freq[i] < 0) {
		  log(Error)<<"Frequency of the sine wave "<< i+1 << " not valid!"<<endlog();
		  return false;
	  }
  }
  
  k = 0;

  return true;
}

void SineWave::updateHook()
{
  Logger::In in("SineWave::updateHook()");
  
  doubles output(vector_size,0.0);
  //double step = 2*PI/256;//remove
  for ( uint i = 0; i < vector_size; i++ ) {
	  output[i] = amplitude[i] * sin(2*PI*freq[i]*k*Ts+phase[i]) + bias[i];
	  //output[i] = floor(output[i]/step+0.5)*step;//remove
  }
  k++;
  
  /*** Write the outputs ***/
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::SineWave)
