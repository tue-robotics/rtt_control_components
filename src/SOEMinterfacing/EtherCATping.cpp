#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "EtherCATping.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATPING;

EtherCATping::EtherCATping(const string& name) : TaskContext(name, PreOperational)
{
	addOperation("sendUpdate", &EtherCATping::sendUpdate, this, OwnThread)
		.doc("Add one or more Encoder ins")
		.arg("N_TIMES","Number of times the warning is displayed");
	
	// Construct Ports
	for ( uint i = 0; i < 7; i++ ) {
		addPort("in"+to_string(i+1), inports[i]);
	}
}
EtherCATping::~EtherCATping(){}

bool EtherCATping::configureHook()
{
	slave_cntrs.resize(N_PORTS);
	countdown_timer = 5;
	return true;
}

bool EtherCATping::startHook()
{
	cntr = 0;
	for ( uint i = 0; i < N_PORTS; i++ ) {
		slave_cntrs[i] = 0;
	}
	
	return true;
}

void EtherCATping::updateHook()
{
	// Check if there is new data on an encoder port
	for ( uint i = 0; i < N_PORTS; i++ ) {
		if ( inports[i].read(in_msg) == NewData) {
			slave_cntrs[i]++;
		}
	}
	
	// Publish Report and reset cntrs
	cntr++;
	if (cntr==1000 && countdown_timer > 0) {		
		// Print Warning
		log(Warning)<<"EtherCATping Report: SLAVE 1: "<<slave_cntrs[1]/10<<" %,SLAVE 2: "<<slave_cntrs[2]/10<<" %,SLAVE 3: "<<slave_cntrs[3]/10<<" %,SLAVE 4: "<<slave_cntrs[4]/10<<" %,SLAVE 5: "<<slave_cntrs[5]/10<<" %,SLAVE 6: "<<slave_cntrs[6]/10<<" %"<<endlog();
		
		// Reset the cntr and the slave_cntrs and update the countdown timer
		countdown_timer--;
		cntr = 0;
		for ( uint i = 0; i < N_PORTS; i++ ) {
			slave_cntrs[i] = 0;
		}
		
	}
}

void EtherCATping::sendUpdate(uint N_TIMES)
{
	countdown_timer = N_TIMES;
	log(Warning)<<"EtherCATping Report Enabled for "<< countdown_timer << "seconds!" <<endlog();

	return;
}

ORO_CREATE_COMPONENT(ETHERCATPING::EtherCATping)
