#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <amigo_ref_interpolator/interpolator.h>

#include "SecondOrderInterpolator.hpp"

/*
 * TODO:
 * See if the rows folluw up. So no: function1, function2, function4
 */


using namespace std;
using namespace RTT;
using namespace AMIGO;

SecondOrderInterpolator::SecondOrderInterpolator(const string& name) : TaskContext(name, PreOperational)
{

  addProperty( "NrInterpolators", NrInterpolators );
  addProperty( "InterpolatorDt", InterpolDt );
  addProperty( "InterpolatorEps", InterpolEps );

}

SecondOrderInterpolator::~SecondOrderInterpolator(){}

bool SecondOrderInterpolator::configureHook()
{
  addPort( "accin", accinport );
  addPort( "velin", velinport );
  addPort( "posin", posinport );
  addPort( "posout", posoutport );
  addPort( "velout", veloutport );
  addPort( "accout", accoutport );
  addPort( "resetValues", resetPort );
  
  mRefGenerators.resize(NrInterpolators);
  desiredAcc.resize(NrInterpolators);
  desiredVel.resize(NrInterpolators);
  desiredPos.resize(NrInterpolators);
  mRefPoints.resize(NrInterpolators);
  //interpolators.resize(NrInterpolators);
  
  return true;

}

bool SecondOrderInterpolator::startHook()
{
  // Check validity of Ports:
  if ( !accinport.connected() || !velinport.connected() || !posinport.connected() )
  {
    log(Error)<<"SecondOrderInterpolator::Inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !posoutport.connected() ) {
    log(Warning)<<"SecondOrderInterpolator::Outputport not connected!"<<endlog();
  }
  if ( !resetPort.connected() ) {
    log(Warning)<<"SecondOrderInterpolator::resetPort not connected!"<<endlog();
  }
  
  return true;
}


void SecondOrderInterpolator::updateHook()
{
  // Read the inputports;
  doubles outacc(NrInterpolators,0.0);
  doubles outvel(NrInterpolators,0.0);
  doubles outpos(NrInterpolators,0.0);
  
  if ( NewData == accinport.read( desiredAcc ) )
    for ( uint i = 0; i < NrInterpolators; i++ )  
	  if ( desiredAcc[i] == 0.0 )
		  log(Warning)<<"Acceleration zero, no movement!"<<endlog();
  if ( NewData == velinport.read( desiredVel ) )
    for ( uint i = 0; i < NrInterpolators; i++ )  
	  if (  desiredVel[i] == 0.0 )
		  log(Warning)<<"Velocity zero, no movement!"<<endlog();
  posinport.read( desiredPos );  
  
    // If new data on the resetport [yes/no, resetpos, resetvel, resetacc] reset the according interpolator(s).
  doubles resetdata(NrInterpolators*4,0.0);
  if (NewData == resetPort.read( resetdata ) ){
	  for ( uint i = 0; i < NrInterpolators; i++ ){
		  if(resetdata[i*4]==1.0){
			  log(Info)<<"ReferenceGenerator: resetting refgen "<<i+1<<"\n"<<endlog();
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
      outacc[i]=mRefPoints[i].acc;
      outvel[i]=mRefPoints[i].vel;
      outpos[i]=mRefPoints[i].pos;
  }
  
  //log(Info)<<"outpos[6] from interpolator = "<<outpos[6]<<endlog();

  accoutport.write( outacc );
  veloutport.write( outvel );
  posoutport.write( outpos );

}

ORO_CREATE_COMPONENT(SecondOrderInterpolator)
