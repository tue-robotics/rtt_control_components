#include "Compare.hpp"
#include <sstream>

using namespace std;
using namespace RTT;
using namespace COMPARE;

Compare::Compare(const string& name) : TaskContext(name, PreOperational)
{
		// Add Ins
    addOperation("AddCompareDoubles", &Compare::AddCompareDoubles, this, OwnThread)
		.doc("Compare a doubles port")
		.arg("SIZE","Size of the ports that should be compared");
		
	addProperty( "warningtime",     	warningtime     ).doc("Time between warnings");    
	addProperty( "nameofcomp",     		nameofcomp     	).doc("name");    

}

Compare::~Compare(){}

bool Compare::configureHook()
{
	n_comparisons_D = 0;
	k_timer = 0;
	
	input_A.resize(MAX_PORTS);
	input_B.resize(MAX_PORTS);
	input_A_msg.resize(MAX_PORTS);
	input_B_msg.resize(MAX_PORTS);
	
	print = false;
	
	return true;
}

bool Compare::startHook(){}

void Compare::updateHook()
{    

	CompareDoubles();
	
	return;
}

void Compare::AddCompareDoubles(double SIZE, bool MSG)
{
	
	// Update properties
	n_comparisons_D++;
	portsizes_D[n_comparisons_D-1] = SIZE;
	
	// Add port
	if (!MSG) {
		addPort( "D"+to_string(n_comparisons_D)+"_A", Ainports_D[n_comparisons_D-1] );
		addPort( "D"+to_string(n_comparisons_D)+"_B", Binports_D[n_comparisons_D-1] );
	} else {
		addPort( "D"+to_string(n_comparisons_D)+"_A", Ainports_D_msg[n_comparisons_D-1] );
		addPort( "D"+to_string(n_comparisons_D)+"_B", Binports_D_msg[n_comparisons_D-1] );
	}
	
	// inputs
	input_A[n_comparisons_D-1].assign( (int) SIZE,0.0);
	input_B[n_comparisons_D-1].assign( (int) SIZE,0.0);
	input_A_msg[n_comparisons_D-1].values.assign( (int) SIZE,0.0);
	input_B_msg[n_comparisons_D-1].values.assign( (int) SIZE,0.0);
	
	msgsstore[n_comparisons_D-1] = MSG;
	
	log(Warning) << "Compare::AddCompareDoubles: Added Doubles compare for: " << nameofcomp << "with size" << SIZE << "." << endlog();

	return;
}

void Compare::CompareDoubles()
{
	std::stringstream blaat;
	
	for( uint i = 0; i < n_comparisons_D; i++ ) {
			
		if (!msgsstore[n_comparisons_D-1]) {

			Ainports_D[i].read(input_A[i]);
			Binports_D[i].read(input_B[i]);

			for( uint j = 0; j < portsizes_D[i]; j++ ) {
				if (fabs(input_A[i][j]-input_B[i][j]) > 0.01) {
					print = true;
					blaat << " [bp=" << i << "],[id=" << j << "]->" << input_A[i][j] << "!=" << input_B[i][j] << "   ";
				}
			}
			
		} else {

			Ainports_D_msg[i].read(input_A_msg[i]);
			Binports_D_msg[i].read(input_B_msg[i]);

			for( uint j = 0; j < portsizes_D[i]; j++ ) {
				if (fabs(input_A_msg[i].values[j]-input_B_msg[i].values[j]) > 0.01) {
					print = true;
					blaat << " [bp=" << i << "],[id=" << j << "]->" << input_A_msg[i].values[j] << "!=" << input_B_msg[i].values[j] << "   ";
				}
			}
		}
		
	}
	
	// Update Timer
	if ( print && k_timer >(warningtime/(this->getPeriod()))) {
		log(Warning) << nameofcomp << ":" << blaat.str() << "!" << endlog();
		k_timer = 0;
		print = false;
	}
	k_timer++;
}

ORO_CREATE_COMPONENT(COMPARE::Compare)
