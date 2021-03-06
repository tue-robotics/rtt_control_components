#include "ReferenceGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

ReferenceGenerator::ReferenceGenerator(const string& name) : TaskContext(name, PreOperational)
{
    // Ports
    addPort( "posout", posoutport );
    addPort( "velout", veloutport );
    addPort( "accout", accoutport );
    addPort( "initial_pos", initialposinport );

    // Attrributes
    addAttribute( "minPosition", minpos );
    addAttribute( "maxPosition", maxpos );
    addAttribute( "maxVelocity", maxvel );

    // Properties
    addProperty( "number_of_inports", N_inports );
    addProperty( "vector_size", N );
    addProperty( "inport_sizes", inport_sizes );
    addProperty( "InterpolatorDt", InterpolDt );
    addProperty( "InterpolatorEps", InterpolEps );
    addProperty( "minPosition", minpos);
    addProperty( "maxPosition", maxpos);
    addProperty( "maxVelocity", maxvel);
    addProperty( "maxAcceleration", maxacc);
    
    // Operations
    addOperation( "resetReference", &ReferenceGenerator::resetReference, this, OwnThread ).doc("Reset the reference generator to measured current position (used in homing)");
}

ReferenceGenerator::~ReferenceGenerator()
{
	//! Remove Operations
	remove("resetReference");
}

bool ReferenceGenerator::configureHook()
{
    // Property Checks
	if ( (N_inports <= 0) || (N_inports > N)) {
        log(Error)<<"Reference Generator: Could not start component: size or N_inports is incorrect. N_inports and N should satisfy (0 < N_inports <= N)"<<endlog();        
        return false;
    }
    if ( (inport_sizes.size() != N_inports) ) {
        log(Error)<<"Reference Generator: Could not start component: Size of inport_sizes is incorrect. should be equal to N_inports"<<endlog();    
        return false;
    }      
	int sumofinputs =0;
    for ( uint j = 0; j < N_inports; j++ ){
		sumofinputs += inport_sizes[j];
	}
	if ( (sumofinputs != N) ) {
        log(Error)<<"Reference Generator: Could not start component: Sum of input_sizes should match N"<<endlog();    
        return false;
    }	    
    if ( (minpos.size() != N) || (maxpos.size() != N) || (maxvel.size() != N) || (maxacc.size() != N) ) {
        log(Error)<<"Reference Generator: Could not start component:  minpos["<< minpos.size() <<"], maxpos["<< maxpos.size() <<"], maxvel["<< maxvel.size() <<"], maxacc["<< maxacc.size() <<"] should be size " << N <<"."<<endlog();        
        return false;
    }
    for ( uint i = 0; i < N; i++ ){
        if ( minpos[i] == 0.0 && maxpos[i] == 0.0 ) {
            log(Warning)<<"Reference Generator: minPos and maxPos both specified 0.0. Thus maxPos and minPos boundaries are not taken into account"<<endlog();
        } else if ( minpos[i] > maxpos[i]) {
            log(Error)<<"Reference Generator: Could not start component: minPosition should be specified smaller than maxPosition"<<endlog();
            return false;
        }
        if ( ( maxvel[i] < 0.0) || ( maxacc[i] < 0.0) ) {
            log(Error)<<"Reference Generator: Could not start component: maxVelocity and maxAcceleration should be specified positive"<<endlog();
            return false;
        }
    }
    
    // add inports
    for ( uint j = 0; j < N_inports; j++ ) {
        addPort( ("posin"+to_string(j+1)), posinport[j] );
	}

    mRefGenerators.resize(N);
    mRefPoints.resize(N);
    desiredPos.assign(N,0.0);
    desiredVel.assign(N,0.0);
    desiredAcc.assign(N,0.0);

    return true;
}

bool ReferenceGenerator::startHook()
{
    // Check validity of Ports:
    for ( uint i = 0; i < N_inports; i++ ){
		if ( (!posinport[i].connected() ) ) {
			log(Warning)<<"ReferenceGenerator:: posin"<< i <<"is not connected!"<<endlog();
		}
	}
    if ( !posoutport.connected() ) {
        log(Warning)<<"Outputport not connected!"<<endlog();
    }


	// Reset the reference generator to the current position.
	// Publishes the current position once.
	resetReference();    

    return true;
}


void ReferenceGenerator::updateHook()
{
    // Read the inputports
    doubles outpos(N,0.0);
    doubles outvel(N,0.0);
    doubles outacc(N,0.0);

    // If new data on the channel then change the desired positions
    // j loops over the number of inports, k loops over the size of the j-th input
    // i loops over the total output size (sum of all input sizes)
    uint i = 0;
    for ( uint j = 0; j < N_inports; j++ ){
		
		doubles inpos(inport_sizes[j],0.0);
		if (NewData == posinport[j].read( inpos ) ){
			
			for ( uint k = 0; k < inport_sizes[j]; k++ ){
				
				if ( minpos[i] == 0.0 && maxpos[i] == 0.0 ) {
					desiredPos[i]=(inpos[k]);
				} else {
					desiredPos[i]=min(inpos[k], maxpos[i]);
					desiredPos[i]=max(minpos[i], desiredPos[i]);
					
					if (inpos[i] < (minpos[i] - 0.05) ) {
						log(Warning) << "ReferenceGenerator: Received goal " << inpos[i] << " on port " << j+1 << ". This is outside minimal bound " << minpos[i] << "!" << endlog();
					}
					if (inpos[i] > (maxpos[i] + 0.05) ) {
						log(Warning) << "ReferenceGenerator: Received goal " << inpos[i] << " on port " << j+1 << ". This is outside maximal bound " << maxpos[i] << "!" << endlog();
					}
				}
				desiredVel[i]=maxvel[i];
				desiredAcc[i]=maxacc[i];	

				i++;
			}
		} else { // if no data received skip 
			i+=inport_sizes[j];
		}
	}

    // Compute the next reference points
    for ( uint i = 0; i < N; i++ ){
        mRefPoints[i] = mRefGenerators[i].generateReference(desiredPos[i], desiredVel[i], desiredAcc[i], InterpolDt, false, InterpolEps);
        outpos[i]=mRefPoints[i].pos;
        outvel[i]=mRefPoints[i].vel;
        outacc[i]=mRefPoints[i].acc;
    }

    posoutport.write( outpos );
    veloutport.write( outvel );
    accoutport.write( outacc );

}

void ReferenceGenerator::resetReference()
{
	// Reset the desired velocity and acceleration
    desiredVel.assign(N,0.0);
    desiredAcc.assign(N,0.0);
    
    //Set the starting value to the current actual value
    doubles actualPos(N,0.0);
    initialposinport.read( actualPos );
    for ( uint i = 0; i < N; i++ ){
        mRefGenerators[i].setRefGen(actualPos[i]);
        desiredPos[i] = actualPos[i];
    }
    
    // Write on the outposport to make sure the receiving components gets new data
    posoutport.write( actualPos );
}

ORO_CREATE_COMPONENT(SOURCES::ReferenceGenerator)
