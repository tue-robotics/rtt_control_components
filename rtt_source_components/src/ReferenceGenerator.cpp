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
  addProperty( "NrInterpolators", NrInterpolators );
  addProperty( "InterpolatorDt", InterpolDt );
  addProperty( "InterpolatorEps", InterpolEps );
}

ReferenceGenerator::~ReferenceGenerator(){}

bool ReferenceGenerator::configureHook()
{
  addPort( "posin", posinport ); //Deprecated
  addPort( "refin", refinport );
  addPort( "posout", posoutport );
  addPort( "velout", veloutport );
  addPort( "accout", accoutport );
  addPort( "actual_pos", actualposinport );
  addEventPort( "resetValues", resetPort );
  
  mRefGenerators.resize(NrInterpolators);
  mRefPoints.resize(NrInterpolators);
  desiredPos.assign(NrInterpolators,0.0);
  desiredVel.assign(NrInterpolators,0.0);
  desiredAcc.assign(NrInterpolators,0.0);  
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
  if ( posinport.connected() )
  {
    log(Warning)<<"ReferenceGenerator::Deprecated port. Use ref_port!"<<endlog();
  }
  if ( !refinport.connected() )
  {
    log(Error)<<"ReferenceGenerator::Inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    //TODO: return false;
  }
  if ( !posoutport.connected() ) {
    log(Warning)<<"ReferenceGenerator::Outputport not connected!"<<endlog();
  }
  if ( resetPort.connected() ) {
    log(Warning)<<"ReferenceGenerator::resetPort is connected! Try to use the service instead."<<endlog();
  }
  
  //Set the starting value to the current actual value
  doubles actualPos(NrInterpolators,0.0);
  actualposinport.read( actualPos );
  for ( uint i = 0; i < NrInterpolators; i++ ){
	  mRefGenerators[i].setRefGen(actualPos[i]);
  }  
  
  //Initialise reference vectors
  refin.resize(NrInterpolators);
  for ( uint i = 0; i < NrInterpolators; i++ ){
	  refin[i].assign(3,0.0); //pos, vel, acc
  }  
  
  // Write on the outposport to make sure the receiving components gets new data
  posoutport.write( actualPos );
  
  log(Warning)<<"ReferenceGenerator::started at " << os::TimeService::Instance()->getNSecs()*1e-9 <<endlog();
  
  return true;
}


void ReferenceGenerator::updateHook()
{
  // Read the inputports
  doubles inpos(NrInterpolators,0.0); //Deprecated
  doubles outpos(NrInterpolators,0.0);
  doubles outvel(NrInterpolators,0.0);
  doubles outacc(NrInterpolators,0.0);
  doubles resetdata(NrInterpolators*4,0.0);
  
  // If new data on the channel then change the desired positions
  if (NewData == refinport.read( refin ) ){
	  for ( uint i = 0; i < NrInterpolators; i++ ){
		  desiredPos[i]=refin[i][0];
		  if ( refin[i][1] != 0.0 ){
			  desiredVel[i]=refin[i][1];
		  }
		  else{
			  desiredVel[i]=interpolators[i][1];
		  }
		  if ( refin[i][2] != 0.0 ){
			  desiredAcc[i]=refin[i][2];
		  }
		  else{
			  desiredAcc[i]=interpolators[i][2];
		  }		 
	  }
  }

  // Deprecated:
  // If new data on the channel then change the desired positions
  if (NewData == posinport.read( inpos ) ){
	  for ( uint i = 0; i < NrInterpolators; i++ ){
		  desiredPos[i]=inpos[i];
		  desiredVel[i]=interpolators[i][1];
 	      desiredAcc[i]=interpolators[i][2];
	  }
	  //log(Warning)<<"New desiredPos"<<endlog();
  } 
  // end Deprecated
  
  // TODO: remove code below, should be a service, not a port, or remove completely
  // If new data on the resetport [yes/no, resetpos, resetvel, resetacc] reset the according interpolator(s).
  if (NewData == resetPort.read( resetdata ) ){
	  for ( uint i = 0; i < NrInterpolators; i++ ){
		  if(resetdata[i*4]==1.0){
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
	  mRefPoints[i] = mRefGenerators[i].generateReference(desiredPos[i], desiredVel[i], desiredAcc[i], InterpolDt, false, InterpolEps);
      outpos[i]=mRefPoints[i].pos;
      outvel[i]=mRefPoints[i].vel;
      outacc[i]=mRefPoints[i].acc;
  }
  
  //log(Info)<<"outpos[6] from interpolator = "<<outpos[6]<<endlog();

  posoutport.write( outpos );
  veloutport.write( outvel );
  accoutport.write( outacc );

}

ORO_CREATE_COMPONENT(SOURCES::ReferenceGenerator)
