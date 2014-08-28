#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "MatrixTransform.hpp"

/*
 * TODO:
 * See if the rows folluw up. So no: function1, function2, function4
 */


using namespace std;
using namespace RTT;
using namespace AMIGO;

MatrixTransform::MatrixTransform(const string& name) : TaskContext(name, PreOperational)
{
  //addProperty( "TransformationMatrix", TransformationMatrix );
  addProperty( "Ncolumns", Ncolumns );
  addProperty( "Nrows", Nrows );


}
MatrixTransform::~MatrixTransform(){}

bool MatrixTransform::configureHook()
{
  for ( uint i = 0; i < Nrows; i++ )
  {
    string name = "function"+to_string(i+1);
    addProperty( name, function[i]);
  }



  /*
  // we make a copy to be allowed to iterate over and extend TransformationMatrix_data:
  PropertyBag bag = TransformationMatrix;

  // Detect if something is in the bag
  if ( bag.empty() )
  {
    log(Error) <<"No transformation matrix given."<<endlog();
    return false;
  }

  Nrows = 0;

  // Define an iterater for all the properties in the bag. The order is not guaranteed to be sequential.
  PropertyBag::const_iterator it = bag.getProperties().begin();

  while ( it != bag.getProperties().end() )
  {
    Property<doubles>* compName = dynamic_cast<Property<doubles>* >( *it );
    string name = compName->getName();
    //log(Debug)<<"name: "<<name<<endlog();

    // Test if the property in the bag is of the correct type
    if ( !compName )
      log(Error) << "Expected Property \"" << (*it)->getName() <<"\" to be of type vector<double>."<< endlog();

    // See if the name of the property is of the right format. ("functionX" Where X is the rownumber)
    else if ( name.compare(0,8,"function") == 0 )
    {
      // Fetch the number of the function
      uint nr = atoi(&name[8]);
      //log(Debug)<<"nr: "<<nr<<endlog();

      // Determine the size, should be equal for all functions
      uint arraylength = compName->value().size();
      //log(Debug)<<"size = "<<arraylength<<endlog();

      // First column size found is the reference
      if ( it == bag.getProperties().begin() )
      {
        Ncolums = arraylength;
        // Check if the number of columns does not exceed hardcoded maximum.
        if (Ncolums > maxN)
        {
          log(Error)<<"You're trying to create "<<Ncolums<<" columns while a maximum of "<<maxN<<" is hardcoded. Appologies"<<endlog();
          return false;
        }
      }
      // Rest should be of equal size
      else if ( Ncolums != arraylength )
      {
        log(Error)<<"Number elements in function"<<nr<<" not equal to "<<Ncolums<<" like the first function found"<<endlog();
        return false;
      }

      // Iterate trough array and add to matrix
      for ( uint i = 0; i<Ncolums; i++ )
      {
        log(Info)<<"matrix["<<nr-1<<"]["<<i<<"] = "<<compName->value()[i]<<endlog();
        matrix[nr-1][i] = compName->value()[i];
      }
      ++Nrows;
      // Check if the number of rows does not exceed hardcoded maximum.
      if (Nrows > maxN)
      {
        log(Error)<<"You're trying to create "<<Nrows<<" rows while a maximum of "<<maxN<<" is hardcoded. Appologies"<<endlog();
        return false;
      }
    }
    else // If the name of the function was wrong:
    {
      log(Error) << "Expected \"function1\", \"function2\" etc., got " << name << endlog();
      return false;
    }
    //log(Debug)<<"++it "<<endlog();
    ++it;
  }
  //log(Debug)<<"matrix reading succeeded"<<endlog();
   */

  // Creating ports:
  addEventPort( "in", inport );
  addPort( "out", outport );

  return true;



}

bool MatrixTransform::startHook()
{
  //log(Debug)<<"function[0][0] = "<<function[0][0]<<endlog();

  /*// Determine the size, should be equal for all functions
  for ( uint i = 0; i <= Nrows; i++ )
  {
    if (Ncolumns != function[i].size())
    {
      log(Error)<<"functions not of equal size! -> "<<Ncolumns<<" != "<<function[i].size()<<endlog();
      return false;
    }
  }*/



  // Check validity of Ports:
  if ( !inport.connected() )
  {
    log(Error)<<"MatrixTransform::Inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !outport.connected() ) {
    log(Warning)<<"MatrixTransform::Outputport not connected!"<<endlog();
  }
  return true;
}


void MatrixTransform::updateHook()
{
  // Read the inputports
  doubles input(Nrows,0.0);
  inport.read( input );

  // Do a matrix multiplication. Elementwise
  doubles output(Nrows,0.0);
  for ( uint i = 0; i < Nrows; i++ )
  {
    output[i] = 0.0;
    for ( uint j = 0; j < Ncolumns; j++ )
    {
      //log(Debug)<<"function["<<i<<"]]"<<j<<"] = "<<function[i][j]<<endlog();
      output[i] += function[i][j] * input[j];
    }
  }

  // Write the outputs
  outport.write( output );
  //log(Debug)<<"First output value = "<<output[0]<<endlog();

}

ORO_CREATE_COMPONENT(MATH::MatrixTransform)
