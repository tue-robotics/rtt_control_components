#include <rtt/Component.hpp>
#include "BenchmarkReference.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

BenchmarkReference::BenchmarkReference(const string& name) :
    TaskContext(name, PreOperational)
{
    addPort( "pos_out", position_outport_ );
    addPort( "vel_out", velocity_outport_ );
    addPort( "eff_out", effort_outport_ );
}

BenchmarkReference::~BenchmarkReference(){}

bool BenchmarkReference::configureHook()
{
	REF0_.assign(N,0.0);
	REF1_.assign(N,0.0);
	REF2_.assign(N,0.0);
	REF3_.assign(N,0.0);
	REF4_.assign(N,0.0);
	REF5_.assign(N,0.0);
	REF6_.assign(N,0.0);
	REF7_.assign(N,0.0);
	REF8_.assign(N,0.0);
	REF9_.assign(N,0.0);
	REF10_.assign(N,0.0);
		
	REF3_[3] = 1.5;	
		
	return true;
}

bool BenchmarkReference::startHook()
{	
	cntr == 0;
    return true;
}

void BenchmarkReference::updateHook()
{
	if (cntr < 1) {
		pos_out_ = REF0_;
	}
	else if ((cntr >= 1) && ( cntr < 2 )) {
		pos_out_ = REF1_;
	}
	else if ((cntr >= 2) && ( cntr < 3 )) {
		pos_out_ = REF2_;
	}
	else if ((cntr >= 3) && ( cntr < 4 )) {
		pos_out_ = REF3_;
	}
	else if ((cntr >= 4) && ( cntr < 5 )) {
		pos_out_ = REF4_;
	}
	else if ((cntr >= 5) && ( cntr < 6 )) {
		pos_out_ = REF5_;
	}
	else if ((cntr >= 6) && ( cntr < 7 )) {
		pos_out_ = REF6_;
	}
	else if ((cntr >= 7) && ( cntr < 8 )) {
		pos_out_ = REF7_;
	}
	else if ((cntr >= 8) && ( cntr < 9 )) {
		pos_out_ = REF8_;
	}
	else if ((cntr >= 9) && ( cntr < 10 )) {
		pos_out_ = REF9_;
	}
	else if ((cntr >= 10) && ( cntr < 11 )) {
		pos_out_ = REF10_;
	}
	else if (cntr >= 11) {
		pos_out_ = REF0_;
	}

	position_outport_.write(pos_out_);
	cntr++;
}

ORO_CREATE_COMPONENT(ROS::BenchmarkReference)
