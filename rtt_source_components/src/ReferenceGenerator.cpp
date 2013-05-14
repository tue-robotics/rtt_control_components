#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <amigo_ref_interpolator/interpolator.h>

#include "ReferenceGenerator.hpp"

/*
 * TODO:
 * See if the rows folluw up. So no: function1, function2, function4
 */


using namespace std;
using namespace RTT;
using namespace AMIGO;

ReferenceGenerator::ReferenceGenerator(const string& name) : TaskContext(name, PreOperational)
{

  addProperty( "NrInterpolators", NrInterpolators );
  addProperty( "InterpolatorDt", InterpolDt );
  addProperty( "InterpolatorEps", InterpolEps );

}

ReferenceGenerator::~ReferenceGenerator(){}

bool ReferenceGenerator::configureHook()
{
  addPort( "posin", posinport );
  addPort( "posout", posoutport );
  addPort( "velout", veloutport );
  addPort( "accout", accoutport );
  addEventPort( "resetValues", resetPort );
  
  mRefGenerators.resize(NrInterpolators);
  desiredPos.resize(NrInterpolators);
  mRefPoints.resize(NrInterpolators);
  //interpolators.resize(NrInterpolators);
  
  for ( uint i = 0; i < NrInterpolators; i++ ){
    string name = "interpolator"+to_string(i+1);
    addProperty( name, interpolators[i]);
  }

  return true;

}

bool ReferenceGenerator::startHook()
{
  // Check validity of Ports:
  if ( !posinport.connected() )
  {
    log(Error)<<"ReferenceGenerator::Inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !posoutport.connected() ) {
    log(Warning)<<"ReferenceGenerator::Outputport not connected!"<<endlog();
  }
  if ( !resetPort.connected() ) {
    log(Warning)<<"ReferenceGenerator::resetPort not connected!"<<endlog();
  }
  
  for ( uint i = 0; i < NrInterpolators; i++ ){
	  // Set the initial desired position to the one stored in the property
	  desiredPos[i] = interpolators[i][0];
  }
  
  return true;
}


void ReferenceGenerator::updateHook()
{
  // Read the inputports
  doubles inpos(NrInterpolators,0.0);
  doubles outpos(NrInterpolators,0.0);
  doubles outvel(NrInterpolators,0.0);
  doubles outacc(NrInterpolators,0.0);
  doubles resetdata(NrInterpolators*4,0.0);
  
  // If new data on the channel then change the desired positions
  if (NewData == posinport.read( inpos ) ){
	  for ( uint i = 0; i < NrInterpolators; i++ ){
		  desiredPos[i]=inpos[i];
	  }
	  //log(Warning)<<"New desiredPos"<<endlog();
  }
  
  // If new data on the resetport [yes/no, resetpos, resetvel, resetacc] reset the according interpolator(s).
  if (NewData == resetPort.read( resetdata ) ){
	  for ( uint i = 0; i < NrInterpolators; i++ ){
		  if(resetdata[i*4]==1.0){
			  log(Warning)<<"ReferenceGenerator: resetting refgen "<<i+1<<"\n"<<endlog();
			  mRefGenerators[i].setRefGen(resetdata[i*4+1]);
			  if(resetdata[i*4+2]!=0.0){
				  interpolators[i][1]=resetdata[i*4+2];
			  }
			  if(resetdata[i*4+3]!=0.0){
				  interpolators[i][2]=resetdata[i*4+3];
			  }
		  }
	  }
  }
  
  // Compute the next reference points
  for ( uint i = 0; i < NrInterpolators; i++ ){
	  mRefPoints[i] = mRefGenerators[i].generateReference(desiredPos[i], interpolators[i][1], interpolators[i][2], InterpolDt, false, InterpolEps);
      outpos[i]=mRefPoints[i].pos;
      outvel[i]=mRefPoints[i].vel;
      outacc[i]=mRefPoints[i].acc;
  }
  
  //log(Info)<<"outpos[6] from interpolator = "<<outpos[6]<<endlog();

  posoutport.write( outpos );
  veloutport.write( outvel );
  accoutport.write( outacc );

}

ORO_CREATE_COMPONENT(ReferenceGenerator)
