/** Reporter.cpp
*
* @class Reporter
*
* \author Tim Clephas
* \date December, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>


#include "Reporter.hpp"

using namespace std;
using namespace RTT;
using namespace CUSTOM;

Reporter::Reporter(const string& name) :
	TaskContext(name, PreOperational),
		N(2)
{
	addProperty( "number_of_inputs", N ).doc("An unsigned integer that specifies the number of input ports");;
	addProperty( "ReportFile", filename ).doc("Name of the report file");;
	addProperty( "FirstLine", firstline ).doc("First line of the file");;
}

Reporter::~Reporter(){}

bool Reporter::configureHook()
{
	Logger::In in("Reporter::configureHook()");

	for ( uint i = 0; i < N; i++ )
	{
		string name_inport = "in"+to_string(i+1);
		addPort( name_inport, inports[i] );
	}
	addEventPort( "trigger", inports[N] );

	return true;
}

bool Reporter::startHook()
{
  Logger::In in("Reporter::startHook()");

  // Check validity of Ports:
  for (uint i = 0; i <= N; i++) {
	  if ( !inports[i].connected() ) {
		  log(Error)<<"Input port "<< i <<" not connected!"<<endlog();
		  // No connection was made, can't do my job !
		  return false;
	  }
  }

  if (N < 1 ) {
    log(Error)<<"Reporter parameters not valid!"<<endlog();
    return false;
  }

  // Create file
  file.open (filename.c_str());
  file << firstline << "\n";

  starttime = os::TimeService::Instance()->getTicks();
  return true;
}

void Reporter::updateHook()
{
  Logger::In in("Reporter::updateHook()");
  stringstream ss;
  doubles output;
  double vector_size = 0;

  for ( uint i = 0; i < N; i++ ) {
	  doubles input;
	  inports[i].read( input );
	  vector_size = input.size();
	  for ( uint j = 0; j < vector_size; j++ ) {
		  output.push_back(input[j]);
	  }
  }


  timestamp = os::TimeService::Instance()->secondsSince( starttime );
  ss << timestamp;
  for ( uint i = 0; i < output.size(); i++ ) {
		ss << " " << output[i];
  }
  ss << "\n";
  file << ss.str();
}

void Reporter::stopHook()
{
	file.close();
}
ORO_CREATE_COMPONENT(CUSTOM::Reporter)
