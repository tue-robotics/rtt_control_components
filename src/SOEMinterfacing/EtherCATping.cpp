#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "EtherCATping.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATPING;

EtherCATping::EtherCATping(const string& name) : TaskContext(name, PreOperational)
{
	for ( uint i = 0; i < 6; i++ ) {
		addPort("in"+to_string(i+1), inports[i]);
	} 
}
EtherCATping::~EtherCATping(){}

bool EtherCATping::configureHook()
{
	slave_cntrs.resize(6);
	return true;
}

bool EtherCATping::startHook()
{
	cntr = 0;
	for ( uint i = 0; i < 6; i++ ) {
		slave_cntrs[i] = 0;
	}
	
	for ( uint i = 0; i < 6; i++ ) {
		if (!inports[i].connected()) {
			log(Warning)<<"EtherCATping Report: Not all 6 Encoder ports are connected"<<endlog();
		}
	}
	
	return true;
}

void EtherCATping::updateHook()
{
	// Check if there is new data on an encoder port
	for ( uint i = 0; i < 6; i++ ) {
		if ( inports[i].read(in_msg) == NewData) {
			slave_cntrs[i]++;
		}
	}
	
	// Publish Report and reset cntrs
	cntr++;
	if (cntr==1000) {		
		for ( uint i = 0; i < 6; i++ ) {
			slave_cntrs[i] = 0;
		}
		cntr = 0;
	}
}

ORO_CREATE_COMPONENT(ETHERCATPING::EtherCATping)
