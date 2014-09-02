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
  addProperty( "time2enc", time2enc ).doc("Amount of encoder inpust that belong to the time value. Example: 2 time, 4 enc then array(2.0,2.0)");
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
  // Determine number of timestamps to be read
  Nt = time2enc.size();
  //SI_value.resize(N);
  //ENC_value.resize(N);
  //init_SI_value.resize(N);
  SI_values.assign(N, 0.0);
  ENC_value.assign(N, 0.0);
  TIME_value.assign(Nt, 0.0);
  init_SI_value.assign(N, 0.0);
  offset.assign(N,0.0);
  previous_time_value.assign(Nt,0.0);
  time_shift.assign(Nt,0.0);

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
  if (Nt > maxN)
  {
      log(Error)<<"You're trying to read "<<Nt<<" encoders while a maximum of "<<maxN<<" is hardcoded."<<endlog();
      return false;
  }
  else if (Nt == 0)
  {
      log(Warning)<<"No connection between time inputs and encoder inputs specified."<<endlog();
      return false;
  }

  log(Info)<<"Creating ports for "<<N<<" encoders."<<endlog();


  // Creating ports
  for ( uint i = 0; i < N; i++ )
  {
    string name_inport = "enc"+to_string(i+1)+"_in";
    if (i != 0) addPort( name_inport, inport_enc[i] );
    else if (i == 0) addPort( name_inport, inport_enc[i] );
  }
  for ( uint i = 0; i < Nt; i++)
  {
      string name_inport = "time"+to_string(i+1)+"_in";
      addPort( name_inport, inport_time[i] );
  }
  addPort( "out", outport );
  addPort( "out_enc", outport_enc );  //TODO: remove
  addPort( "out_time", outport_time ); //TODO: remove
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
  for ( uint i = 0; i <Nt; i++){
      if ( !inport_time[i].connected() ) {
          log(Warning)<<"ReadEncodersTimeStamp::Inputport timeSt " << i << " not connected!"<<endlog();
      }
  }


  //initialize time variables
  EncoderMsg timedata;
  for ( uint i = 0; i < Nt; i++ )
  {
      inport_time[i].read(timedata);
      previous_time_value[i] = timedata.value;
      time_shift[i] = 0.0;
  }

  EncoderMsg encdata;
  for ( uint i = 0; i < N; i++ )
  {
    // Initialising variables
    inport_enc[i].read(encdata);
    ienc[i] = 0;
    previous_enc_position[i] = encdata.value; // obsolete
    init_SI_value[i] = (double)previous_enc_position[i] * enc2SI[i] + offset[i];
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

  // update encoder values
  uint time_i = 0;
  double time_switch = time2enc[time_i];
  time_correction = readTime(time_i);
  for ( uint i = 0; i < N; i++ ) {
      // check if time should be changed
      if ( i >= time_switch) {
          time_i ++;
          time_correction = readTime(time_i);
          time_switch = time_switch + time2enc[time_i];
      }
      // get estimated encoder values
      SI_values[i] = readEncoder(i);
  }
  
  outport.write(SI_values);
  outport_enc.write(ENC_value);
  outport_time.write(TIME_value);
  counter++;
  
  if ( counter >= 5000 ) {
	  //log(Warning) << "time_correction = " << time_correction << endlog();
      counter = 0;
  }
}

double ReadEncodersTimeStamp::readEncoder( int i )
{
  // get the encoder data
  EncoderMsg encdata;
  if ( inport_enc[i].read(encdata) != NewData ) {
      log(Warning) << " No new data recieved on encoder " << i << endlog();
      int enc_position = ienc[i] * encoderbits + previous_enc_position[i];
      return ((double)enc_position * enc2SI[i]) - init_SI_value[i] + offset[i];
  }
  uint new_enc_position = encdata.value;
  ENC_value[i] = encdata.value;
  
  int enc_position_previous = ienc[i] * encoderbits + previous_enc_position[i]; 
  // correct for value overflow
  if( ((double)previous_enc_position[i] - new_enc_position) > encoderbits/2)
    ienc[i]++;
  else if( ((double)previous_enc_position[i] - new_enc_position) < (-1 * (double)encoderbits/2))
    ienc[i]--;


  // calculate position
  int enc_position = ienc[i] * encoderbits + new_enc_position;  
  double enc_position_corrected = (double)enc_position_previous + ((double)enc_position - (double)enc_position_previous) * time_correction;
  double SI_value =  (enc_position_corrected * enc2SI[i]) - init_SI_value[i] + offset[i];
  previous_enc_position[i] = new_enc_position;
  
  if (SI_value > 10000){
	  log(Warning) << " enc_position_previous" << endlog();
  }

  // return position
  return SI_value;
}

double ReadEncodersTimeStamp::readTime( int i )
{
    // get time data
    EncoderMsg timedata;
    //inport_time.read(timedata);
    if ( inport_time[i].read(timedata) != NewData ) {
        timedata.value = previous_time_value[i]+Ts/timestep;
        log(Warning) << " No new data recieved on time input, set to " << timedata.value << " previous = " << previous_time_value[i] << endlog();
    }
    
    uint new_time_value = timedata.value;
    double time_value = timedata.value;
    TIME_value[i] = timedata.value;

    // calculate the time correction value
    double time_diff;
    double time_cor;
    if ( (previous_time_value[i] - new_time_value) > timebits/2) {
        time_diff = (time_value + (double)timebits - previous_time_value[i]) * timestep;
    } else {
        time_diff = (time_value - previous_time_value[i]) * timestep;
    }
    time_cor = (Ts - time_shift[i]) / time_diff;

    // update paramters
    time_shift[i] = time_shift[i] + time_diff - Ts;
    previous_time_value[i] = time_value;

    // return correction paramter
    return time_cor;
}

void ReadEncodersTimeStamp::reset( uint Nreset, double resetvalue )
{
  // ReInitialising variables
  ienc[Nreset] = 0;
  EncoderMsg encdata;
  inport_enc[Nreset].read(encdata);
  ienc[Nreset] = 0;
  previous_enc_position[Nreset] = encdata.value; // obsolete
  init_SI_value[Nreset] = (double)previous_enc_position[Nreset] * enc2SI[Nreset] + offset[Nreset] - resetvalue;
  if (Nreset == 1)
    log(Warning)<<"ReadEncodersTimeStamp: Nulling encoders"<<endlog();
}

ORO_CREATE_COMPONENT(SOEM::ReadEncodersTimeStamp)
