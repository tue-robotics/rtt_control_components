/** RealTimeDerivator.cpp
 *
 * @class RealTimeDerivator
 *
 * \author Sava Marinkov
 * \date May, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "RealTimeDerivator.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

RealTimeDerivator::RealTimeDerivator(const string& name) : TaskContext(name, PreOperational) {
  addPort("u", inport);
  addPort("ue", ue_port);  
  addPort("ude", ude_port);  
  addPort("udde", udde_port);
  addPort("derivatives", msg_port);
  addProperty( "vector_size", vector_size ).doc("Specifies the size of the input and output vectors");
  addProperty( "bw", f).doc("Bandwidth of the filter in Hz");
}

RealTimeDerivator::~RealTimeDerivator(){}

bool RealTimeDerivator::configureHook() {
	return true;
}

bool RealTimeDerivator::startHook() {
	if (vector_size>MAX_SIZE) vector_size=MAX_SIZE;

	k1 = (2*PI*f)*(2*PI*f);
	k2 = 0.707*2*PI*f;
	for (int i=0; i<vector_size; i++) {
		ue_prev[i] = 0.0;
		ude_prev[i] = 0.0;
		udde_prev[i]= 0.0;
	}
	
	old_time = os::TimeService::Instance()->getNSecs()*1e-9;
	return true;
}

void RealTimeDerivator::updateHook()
{
	doubles u(vector_size,0.0), ue(vector_size,0.0), ued(vector_size,0.0), uedd(vector_size,0.0);
	
	std_msgs::Float32MultiArray msgx;
	msgx.data.resize(3*vector_size);
	msgx.layout.dim.resize(2);
	msgx.layout.dim[0].label  = "channel";
	msgx.layout.dim[0].size   = vector_size;
	msgx.layout.dim[0].stride = 3*vector_size;
	msgx.layout.dim[1].label  = "derivative";
	msgx.layout.dim[1].size   = 3;
	msgx.layout.dim[1].stride = vector_size;
	msgx.layout.data_offset = 0;
	
	inport.read(u);
	double Ts = determineDt();
	
	for (int i=0; i<vector_size; i++) {
		ue[i] = ue_prev[i] + Ts*ude_prev[i] + Ts*Ts*udde_prev[i];
		ued[i] = ude_prev[i] + Ts*udde_prev[i];
		uedd[i] = k1*(u[i]-ue[i])-k2*ued[i];

		msgx.data[i] = ue[i];
		msgx.data[i+vector_size] = ued[i];
		msgx.data[i+2*vector_size] = uedd[i];
	}

	for (int i=0; i<vector_size; i++) {
		ue_prev[i] = ue[i];
		ude_prev[i] = ued[i];
		udde_prev[i] = uedd[i];	
	}

	msg_port.write(msgx);
	ue_port.write(ue);
	ude_port.write(ued);
	udde_port.write(uedd);
}

double RealTimeDerivator::determineDt()
{
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  double dt = (double)(new_time - old_time);
  old_time = new_time;
  return dt;
}

ORO_CREATE_COMPONENT(MATH::RealTimeDerivator)
