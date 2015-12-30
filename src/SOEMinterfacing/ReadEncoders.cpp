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
    addProperty( "offset", offsets ).doc("Offset value in SI units, untested feature");
    addOperation( "ResetEncoders", &ReadEncoders::ResetEncoders, this, OwnThread ).doc("Reset an encoder value to a new value, usefull for homing");
}
ReadEncoders::~ReadEncoders()
{
	//! Remove Operations
	remove("ResetEncoders");
}

bool ReadEncoders::configureHook() 
{
    // Determine number of encoders to be read
    N = enc2SI.size();
    SI_values.assign(N, 0.0);
    ENC_values.assign(N, 0.0);
    init_SI_values.assign(N, 0.0);
    offsets.assign(N,0.0);
    enc_position.assign(N,0);
    enc_position_prev.assign(N,0);
    enc_velocity.assign(N,0.0);

    if (N > maxN) {
        log(Error)<<"You're trying to read "<<N<<" encoders while a maximum of "<<maxN<<" is hardcoded. Appologies"<<endlog();
        return false;
    } else if (N == 0) {
        log(Warning)<<"No conversion factors given (enc2SI), make sure you're loading the properties ok"<<endlog();
        return false;
    }
    log(Info)<<"Creating ports for "<<N<<" encoders."<<endlog();

    // Creating ports
    for ( uint i = 0; i < N; i++ ) {
        string name_inport = "enc"+to_string(i+1)+"_in";
        if (i != 0) {
            addPort( name_inport, inport_enc[i] );
        } else if (i == 0) {
            addEventPort( name_inport, inport_enc[i] );
        }
    }

    addPort( "out", outport );
    addPort( "out_enc", outport_enc );
    addPort( "in_reNull", inport_reNull );
    addPort( "in_init", inport_init );
    addPort( "vel", outport_vel );

    counter = 0;

    return true;
}

bool ReadEncoders::startHook()
{
    // Check validity of Ports:
    for ( uint i = 0; i < N; i++ ) {
        if ( !inport_enc[i].connected() ) {
            log(Error)<<"ReadEncoders:: Could not start component: Inputport not connected!"<<endlog();
            return false;
        }
    }
    if ( !outport.connected() ) {
        log(Warning)<<"ReadEncoders::Outputport not connected!"<<endlog();
    }

    for ( uint i = 0; i < N; i++ ) {
        // Initialising variables
        ienc[i] = 0;
        previous_enc_position[i] = 0.0; // obsolete
        init_SI_values[i] = 0.0;
        init_SI_values[i] = readEncoder(i)-offsets[i];
        enc_position_prev[i] = enc_position[i];
    }

    return true;
}

void ReadEncoders::updateHook()
{
    bool reNull;

    if(NewData == inport_reNull.read(reNull)){
        if(reNull == true){
            log(Info)<<"ReadEncoders: Renull signal received"<<endlog();
            doubles zeros(N,0.0);
            ResetEncoders(zeros);
            reNull = false;
        }
    }

    // Reset the encoders with an initial offset.
    doubles reset_values;
    if( NewData == inport_init.read(reset_values)){
        if(reset_values.size() == N){
            log(Info)<<"ReadEncoders: initialize signal received"<<endlog();
            ResetEncoders(reset_values);
        }
    }

    // determine dt
    double dt = determineDt();

    for ( uint i = 0; i < N; i++ ) {
        SI_values[i] = readEncoder(i);
        enc_velocity[i] = ((double)(enc_position[i]-enc_position_prev[i])*enc2SI[i])/dt;
        enc_position_prev[i] = enc_position[i];
    }

    outport.write(SI_values);
    outport_vel.write(enc_velocity);
    outport_enc.write(ENC_values);
    counter++;
}

double ReadEncoders::readEncoder( int i )
{
    EncoderMsg encdata;
    inport_enc[i].read(encdata);

    uint new_enc_position = encdata.value;
    ENC_values[i] = encdata.value;
    if( (previous_enc_position[i] - new_enc_position) > encoderbits/2)
        ienc[i]++;
    else if( (previous_enc_position[i] - new_enc_position) < (-1 * (double)encoderbits/2))
        ienc[i]--;
    previous_enc_position[i] = new_enc_position;
    enc_position[i] = ienc[i] * encoderbits + new_enc_position;
    double SI_value =  ((double)enc_position[i] * enc2SI[i]) - init_SI_values[i] + offsets[i];
    return SI_value;
}

double ReadEncoders::determineDt()
{
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    double dt = (new_time - old_time);
    old_time = new_time;
    return dt;
}

void ReadEncoders::ResetEncoders( doubles resetvalues )
{
	if (resetvalues.size() != N) {
		log(Error)<<"ReadEncoders::ResetEncoders: initialize signal received with incorrect size: " << resetvalues.size() << ". Size should be " << N <<"."<<endlog();
	}
	
	for ( uint i = 0; i < resetvalues.size(); i++ ) {
		ienc[i] = 0;
		previous_enc_position[i] = 0.0;
		init_SI_values[i] = 0.0;
		init_SI_values[i] = readEncoder(i) - resetvalues[i];
		enc_position_prev[i] = enc_position[i];
		
		//if (i == 0) { log(Warning)<<"ReadEncoders::ResetEncoders: Resetting Encoders. init_SI_values[i] = readEncRes - resetvalues[i]; -> : " << init_SI_values[0] << " = " << readEncRes  << " - " << resetvalues[0]  << "!"<<endlog(); }
	}
}

ORO_CREATE_COMPONENT(SOEM::ReadEncoders)
