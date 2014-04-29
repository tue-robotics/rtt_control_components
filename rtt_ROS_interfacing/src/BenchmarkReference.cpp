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
		
		
	// step 1 q12
	REF1_[0] = -1.5;
	REF1_[1] =  0.0;
	
	REF2_[0] = -1.5;
	REF2_[1] = -1.5;
	
	REF3_[0] = -1.5;
	REF3_[1] =  1.5;
	
	// step 3 q345
	
	REF5_[2] = 0.0;
	REF5_[3] = 1.5;
	REF5_[4] = -1.5;
	
	REF6_[2] = -1.5;
	REF6_[3] = 1.5;
	REF6_[4] = -1.5;
		
	// step 3 q67
	
	REF8_[5] =  0.5;
	REF8_[6] =  0.0;
		
	REF9_[5] =  0.0;
	REF9_[6] =  -0.5;
	
	return true;
}

bool BenchmarkReference::startHook()

{	
	cntr = 0;
    return true;
}

void BenchmarkReference::updateHook()
{
	log(Warning)<< "UPDATEHOOK BENCHMARK TOOL" <<endlog();
	if (cntr < 1) {
		pos_out_ = REF0_;
		log(Warning)<< " Publishing REF0 " <<endlog();
	}
	else if ((cntr >= 1) && ( cntr < 2 )) {
		pos_out_ = REF1_;
		log(Warning)<< " Publishing REF1 " <<endlog();
	}
	else if ((cntr >= 2) && ( cntr < 3 )) {
		pos_out_ = REF2_;
		log(Warning)<< " Publishing REF2 " <<endlog();
	}
	else if ((cntr >= 3) && ( cntr < 4 )) {
		pos_out_ = REF3_;
		log(Warning)<< " Publishing REF3 " <<endlog();
	}
	else if ((cntr >= 4) && ( cntr < 5 )) {
		pos_out_ = REF4_;
		log(Warning)<< " Publishing REF4 " <<endlog();
	}
	else if ((cntr >= 5) && ( cntr < 6 )) {
		pos_out_ = REF5_;
		log(Warning)<< " Publishing REF5 " <<endlog();
	}
	else if ((cntr >= 6) && ( cntr < 7 )) {
		pos_out_ = REF6_;
		log(Warning)<< " Publishing REF6 " <<endlog();
	}
	else if ((cntr >= 7) && ( cntr < 8 )) {
		pos_out_ = REF7_;
		log(Warning)<< " Publishing REF7 " <<endlog();
	}
	else if ((cntr >= 8) && ( cntr < 9 )) {
		pos_out_ = REF8_;
		log(Warning)<< " Publishing REF8 " <<endlog();
	}
	else if ((cntr >= 9) && ( cntr < 10 )) {
		pos_out_ = REF9_;
		log(Warning)<< " Publishing REF9 " <<endlog();
	}
	else if ((cntr >= 10) && ( cntr < 11 )) {
		pos_out_ = REF10_;
		log(Warning)<< " Publishing REF10 " <<endlog();
	}
	else if (cntr >= 11) {
		pos_out_ = REF0_;
		log(Warning)<< " Publishing REF0 (11)" <<endlog();
		cntr--;
	}

	position_outport_.write(pos_out_);
	cntr++;
}

ORO_CREATE_COMPONENT(ROS::BenchmarkReference)
