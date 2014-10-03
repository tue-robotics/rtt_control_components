#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <amigo_ref_interpolator/interpolator.h>

#include "ReferenceGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

ReferenceGenerator::ReferenceGenerator(const string& name) : TaskContext(name, PreOperational)
{
    addProperty( "vector_size", N );
    addProperty( "InterpolatorDt", InterpolDt );
    addProperty( "InterpolatorEps", InterpolEps );
    addProperty( "MaxPosition", maxpos);
    addProperty( "MaxVelocity", maxvel);
    addProperty( "MaxAcceleration", maxacc);
}

ReferenceGenerator::~ReferenceGenerator(){}

bool ReferenceGenerator::configureHook()
{
    addPort( "posin", posinport );
    addPort( "posout", posoutport );
    addPort( "velout", veloutport );
    addPort( "accout", accoutport );
    addPort( "actual_pos", actualposinport );

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
  if ( (!posinport.connected() ) ) {
    log(Warning)<<"ReferenceGenerator::No inputport connected! connect posin"<<endlog();
  }
  if ( !posoutport.connected() ) {
    log(Warning)<<"ReferenceGenerator::Outputport not connected!"<<endlog();
  }
  
  //Set the starting value to the current actual value
  doubles actualPos(N,0.0);
  actualposinport.read( actualPos );
  for ( uint i = 0; i < N; i++ ){
	  mRefGenerators[i].setRefGen(actualPos[i]);
  }  
  
  // Write on the outposport to make sure the receiving components gets new data
  posoutport.write( actualPos );
  
  log(Info)<<"ReferenceGenerator::started at " << os::TimeService::Instance()->getNSecs()*1e-9 <<endlog();
  
  return true;
}


void ReferenceGenerator::updateHook()
{
    // Read the inputports
    doubles inpos(N,0.0);
    doubles outpos(N,0.0);
    doubles outvel(N,0.0);
    doubles outacc(N,0.0);
    doubles resetdata(N*4,0.0);

    // If new data on the channel then change the desired positions
    if (NewData == posinport.read( inpos ) ){
        for ( uint i = 0; i < N; i++ ){
            if (maxpos[i] == 0.0 ) {
                desiredPos[i]=(inpos[i]);
            } else {
                desiredPos[i]=min(inpos[i], maxpos[i]);
            }
            desiredVel[i]=maxvel[i];
            desiredAcc[i]=maxacc[i];
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

ORO_CREATE_COMPONENT(SOURCES::ReferenceGenerator)
