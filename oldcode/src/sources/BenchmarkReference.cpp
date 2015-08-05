#include <rtt/Component.hpp>
#include "BenchmarkReference.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

BenchmarkReference::BenchmarkReference(const string& name) :
    TaskContext(name, PreOperational)
{
    addProperty( "vector_size", N );
    addPort( "pos_out", position_outport );
}

BenchmarkReference::~BenchmarkReference(){}

bool BenchmarkReference::configureHook()
{
	// check vectorsize
	if (N > maxN) {
		log(Warning)<< "BenchmarkReference: Could not configure Component. N is larger than maxN!" <<endlog();
		return false;
	}
	
	for (uint i = 0; i < N; i++) {
		string name = "referenceFunction"+to_string(i+1);
		addProperty( name, referenceFunction[i]);
	}
	addProperty( "timeFunction", timeFunction);
	
	return true;
}

bool BenchmarkReference::startHook()
{
	int firstInputSize = referenceFunction[0].size();
	for (uint i = 1; i < N; i++) {
		if (referenceFunction[i].size() != firstInputSize) {
			log(Warning)<< "BenchmarkReference: Could not start Component. Size of referenceFunction" << i << " is unequal to the size of referenceFunction1. All referenceFunctions should have the same size!" <<endlog();
			return false;
		}
	}
	if (timeFunction.size() != firstInputSize) {
		log(Warning)<< "BenchmarkReference: Could not start Component. Size of timeFunction is unequal to the size of the referenceFunctions." <<endlog();
		return false;
	}
	
	cntr_ms = 0;
	cntr_s = 0;
	endOfReferenceReached = false;
	timeFunction_j = 0;
	pos_out.assign(N,0.0);
	
    return true;
}

void BenchmarkReference::updateHook()
{
	// Do the Counter Magic
	if (!endOfReferenceReached) {
		cntr_ms++;
		if (cntr_ms >= 1000 ) {
			cntr_ms = 0;
			cntr_s++;
		}
		if (cntr_s >= timeFunction[timeFunction.size()-1]) {
			endOfReferenceReached = true;
			pos_out.assign(N,0.0);
			position_outport.write(pos_out);
		}
		
		// Update pos_out if necessary
		if ( cntr_s >= timeFunction[timeFunction_j]) {
			for (uint i = 0; i < N; i++) {
				pos_out[i] = referenceFunction[i][timeFunction_j];
			}
			position_outport.write(pos_out);
			timeFunction_j++;
		}
		
	} else {
		pos_out.assign(N,0.0);
		position_outport.write(pos_out);
	}
}

ORO_CREATE_COMPONENT(SOURCES::BenchmarkReference)
