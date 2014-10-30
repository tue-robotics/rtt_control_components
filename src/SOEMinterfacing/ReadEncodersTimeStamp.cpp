#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#include "ReadEncodersTimeStamp.hpp"


using namespace std;
using namespace RTT;
using namespace soem_beckhoff_drivers;
using namespace SOEM;

ReadEncodersTimeStamp::ReadEncodersTimeStamp(const string& name) : TaskContext(name, PreOperational)
{
  // Creating Properties
  addProperty( "encoderbits", encoderbits ).doc("Saturation value of the encoder. For example: 65536 for a 16 bit encoder");
  addProperty( "enc2SI", enc2SI ).doc("Value to convert the encoder value to an SI value. Typically 2pi/(encodersteps_per_rev*gearbox)");
  addProperty( "offset", offset ).doc("Offset value in SI units, untested feature");
  addProperty( "timestep", timestep ).doc("time of 1 step in the time input value in s. For example: 0.000000256 for a step size of 256 ns");
  addProperty( "timebits", timebits ).doc("Saturation value of the time. For example: 65536 for a 16 bit time signal");
  addOperation( "reset", &ReadEncodersTimeStamp::reset, this, OwnThread ).doc("Reset an encoder value to a new value, usefull for homing");
}
ReadEncodersTimeStamp::~ReadEncodersTimeStamp(){}

bool ReadEncodersTimeStamp::configureHook()
{
  // get the sample period
  Ts = getPeriod();
  if (Ts <= 0.0)
  {
      log(Error)<<"ReadEncodersTimeStamp, sample time is invallid."<<endlog();
  }

  // check the time settings
  if ( timestep <= 0.0 ) {
      log(Error)<<"ReadEncodersTimeStamp, timestep is invallid."<<endlog();
  }
  if ( timebits <= 0.0 ) {
      log(Error)<<"ReadEncodersTimeStamp, timebits is invallid."<<endlog();
  }

  // Determine number of encoders to be read
  N = enc2SI.size();
  //SI_value.resize(N);
  //ENC_value.resize(N);
  //init_SI_value.resize(N);
  SI_values.assign(N, 0.0);
  ENC_value.assign(N, 0.0);
  time_value = 0.0;
  init_SI_value.assign(N, 0.0);
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
  addPort( "in_time", inport_time);
  addPort( "out_enc", outport_enc );  //TODO: remove
  addPort( "in_reNull", inport_reNull );  //TODO: remove
  
  counter = 0;

  return true;
}

bool ReadEncodersTimeStamp::startHook()
{
  // Check validity of Ports:
  for ( uint i = 0; i < N; i++ )
  {
    if ( !inport_enc[i].connected() )
    {
      log(Error)<<"ReadEncodersTimeStamp::Inputport not connected!"<<endlog();
      return false;
    }
  }
  if ( !outport.connected() ) {
    log(Warning)<<"ReadEncodersTimeStamp::Outputport not connected!"<<endlog();
  }
  if ( !inport_time.connected() ) {
      log(Warning)<<"ReadEncodersTimeStamp::Inputport timeSt not connected!"<<endlog();
  }

  //initialize time value
  EncoderMsg timedata;
  inport_time.read(timedata);
  time_value = timedata.value;
  previous_time_value = time_value;

  EncoderMsg encdata;
  for ( uint i = 0; i < N; i++ )
  {
    // Initialising variables
    inport_enc[i].read(encdata);
    ienc[i] = 0;
    previous_enc_position[i] = encdata.value; // obsolete
    init_SI_value[i] = previous_enc_position[i] * enc2SI[i] + offset[i];
  }

  return true;
}

void ReadEncodersTimeStamp::updateHook()
{
  bool reNull;
		
  if(NewData == inport_reNull.read(reNull)){
    if(reNull == true){
      log(Info)<<"ReadEncodersTimeStamp: Renull signal received"<<endlog();
        for ( uint i = 0; i < N; i++ ) {
          reset(i, 0.0);
	    }
      reNull = false;
    }
  }
	
  for ( uint i = 0; i < N; i++ ) {
	SI_values[i] = readEncoder(i);
  }
    
  outport.write(SI_values);
  outport_enc.write(ENC_value);
  counter++;
}

double ReadEncodersTimeStamp::readEncoder( int i )
{
  // get the encoder data
  EncoderMsg encdata;
  if ( inport_enc[i].read(encdata) != NewData ) {
      log(Warning) << " No new data recieved on encoder " << i << endlog();
  }
  uint new_enc_position = encdata.value;
  ENC_value[i] = encdata.value;
  // get the time
  double time_correction = readTime();

  // correct for value overflow
  if( (previous_enc_position[i] - new_enc_position) > encoderbits/2)
    ienc[i]++;
  else if( (previous_enc_position[i] - new_enc_position) < (-1 * (double)encoderbits/2))
    ienc[i]--;


  // calculate position
  int enc_position = ienc[i] * encoderbits + new_enc_position;
  double enc_position_corrected = previous_enc_position[i] + ((double)enc_position - previous_enc_position[i]) * time_correction;
  double SI_value =  (enc_position_corrected * enc2SI[i]) - init_SI_value[i] + offset[i];
  previous_enc_position[i] = new_enc_position;

  // return position
  return SI_value;
}

double ReadEncodersTimeStamp::readTime()
{
    // get time data
    EncoderMsg timedata;
    if ( inport_time.read(timedata) != NewData ) {
        log(Warning) << " No new data recieved on time input" << endlog();
    }
    uint new_time_value = timedata.value;
    time_value = timedata.value;

    // calculate the time correction value
    double time_diff;
    double time_correction;
    if ( (previous_time_value - new_time_value) > timebits/2) {
        time_diff = (time_value + (double)timebits - previous_time_value) * timestep;
    } else {
        time_diff = (time_value - previous_time_value) * timestep;
    }
    time_correction = (Ts - time_shift) / time_diff;

    // update paramters
    time_shift = time_shift + time_diff - Ts;
    previous_time_value = time_value;

    // return correction paramter
    return time_correction;
}

void ReadEncodersTimeStamp::reset( uint Nreset, double resetvalue )
{
  // ReInitialising variables
  ienc[Nreset] = 0;
  EncoderMsg encdata;
  inport_enc[Nreset].read(encdata);
  ienc[Nreset] = 0;
  previous_enc_position[Nreset] = encdata.value; // obsolete
  init_SI_value[Nreset] = previous_enc_position[Nreset] * enc2SI[Nreset] + offset[Nreset] - resetvalue;
  if (Nreset == 1)
    log(Warning)<<"ReadEncodersTimeStamp: Nulling encoders"<<endlog();
}

ORO_CREATE_COMPONENT(SOEM::ReadEncodersTimeStamp)
