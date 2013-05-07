#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#include "ReadEncoders.hpp"


using namespace std;
using namespace RTT;
using namespace soem_beckhoff_drivers;
using namespace SOEM;

ReadEncoders::ReadEncoders(const string& name) : TaskContext(name, PreOperational)
{
  // Creating Properties
  addProperty( "encoderbits", encoderbits ).doc("Saturation value of the encoder. For example: 65536 for a 16 bit encoder");
  addProperty( "enc2SI", enc2SI ).doc("Value to convert the encoder value to an SI value. Typically 2pi/(encodersteps_per_rev*gearbox)");
  addProperty( "offset", offset ).doc("Offset value in SI units, untested feature");
  addOperation( "reset", &ReadEncoders::reset, this, OwnThread ).doc("Reset an encoder value to a new value, usefull for homing");
}
ReadEncoders::~ReadEncoders(){}

bool ReadEncoders::configureHook()
{
  // Determine number of encoders to be read
  N = enc2SI.size();
  SI_value.resize(N);
  init_SI_value.resize(N);
  offset.assign(N,0.0);

  if (N > maxN)
  {
    log(Error)<<"You're trying to read "<<N<<" encoders while a maximum of "<<maxN<<" is hardcoded. Appologies"<<endlog();
    return false;
  }
  else if (N == 0)
  {
    log(Warning)<<"No conversion factors given (enc2SI), make sure you're loading the properties ok"<<endlog();
    return false;
  }
  log(Info)<<"Creating ports for "<<N<<" encoders."<<endlog();


  // Creating ports
  for ( uint i = 0; i < N; i++ )
  {
    string name_inport = "enc"+to_string(i+1)+"_in";
    if (i != 0) addPort( name_inport, inport_enc[i] );
    else if (i == 0) addEventPort( name_inport, inport_enc[i] );
  }
  addPort( "out", outport );

  counter = 0;

  return true;
}

bool ReadEncoders::startHook()
{
  // Check validity of Ports:
  for ( uint i = 0; i < N; i++ )
  {
    if ( !inport_enc[i].connected() )
    {
      log(Error)<<"ReadEncoders::Inputport not connected!"<<endlog();
      return false;
    }
  }
  if ( !outport.connected() ) {
    log(Warning)<<"ReadEncoders::Outputport not connected!"<<endlog();
  }
  
  sleep(1); // This is to ensure soem is already sending correct data. Might be much less than one second.
  for ( uint i = 0; i < N; i++ )
  {
    // Initialising variables
    ienc[i] = 0;
    previous_enc_position[i] = 0.0; // obsolete
    init_SI_value[i] = 0.0;
    init_SI_value[i] = readEncoder(i);
  }
  return true;
}

void ReadEncoders::updateHook()
{
  for ( uint i = 0; i < N; i++ )
    SI_value[i] = readEncoder(i);
  outport.write(SI_value);
  counter++;
}

double ReadEncoders::readEncoder( int i )
{

  EncoderMsg encdata;
  inport_enc[i].read(encdata);
  uint new_enc_position = encdata.value;
  if( (previous_enc_position[i] - new_enc_position) > encoderbits/2)
    ienc[i]++;
  else if( (previous_enc_position[i] - new_enc_position) < (-1 * (double)encoderbits/2))
    ienc[i]--;
  previous_enc_position[i] = new_enc_position;
  int enc_position = ienc[i] * encoderbits + new_enc_position;
  double SI_value =  ((double)enc_position * enc2SI[i]) - init_SI_value[i] + offset[i];
  return SI_value;
}

void ReadEncoders::reset( uint Nreset, double resetvalue )
{
	// ReInitialising variables
  ienc[Nreset] = 0;
  previous_enc_position[Nreset] = 0.0; // obsolete
  init_SI_value[Nreset] = 0.0;
  init_SI_value[Nreset] = readEncoder(Nreset) - resetvalue;
}

ORO_CREATE_COMPONENT(SOEM::ReadEncoders)
