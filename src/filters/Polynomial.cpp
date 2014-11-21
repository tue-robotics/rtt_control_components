/** Polynomials.cpp
 *
 * @class Polynomials
 *
 * \author Janno Lunenburg
 * \date December, 2013
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Polynomial.hpp"

using namespace std;
using namespace RTT;
using namespace FILTERS;

Polynomials::Polynomials(const string& name) :
    TaskContext(name, PreOperational),
    vector_size(0)
{

    addProperty( "vector_size", vector_size ).doc("Size of the input vector.");
    addProperty( "orders", orders ).doc("Orders of the various polynomials, length of orders should be equal to vector_size");

}

Polynomials::~Polynomials(){}

bool Polynomials::configureHook()
{
    
    if (vector_size != orders.size())
    {
        log(Error)<<"Vector size ("<<vector_size<<") does not correspond with size of vector with orders ("<<orders.size()<<")"<<endlog();
        return false;
    }

    // Adding ports
    addEventPort( "in", inport );
    addPort( "out", outport );

    /* Guaranteeing Real-Time data flow */
    // create an example data sample of vector_size:
    doubles example(vector_size, 0.0);

    // show it to the port (this is a not real-time operation):
    outport.setDataSample( example );
    /* Guaranteeing Real-Time data flow */

    /// Initialize filters
    coefficients.resize(vector_size);
    for (unsigned int i = 0; i < vector_size; i++)
    {
        /// Initialize filter
        Polynomial polynomial(orders[i]);

        /// Initialize coefficients properie
        coefficients[i].resize(orders[i]+1);
        string property_name = "coefficients"+to_string(i+1);
        addProperty( property_name, coefficients[i] );

        /// Pushback filter
        polynomials.push_back(polynomial);
    }

    input.assign(vector_size, 0.0);
    output.assign(vector_size, 0.0);

    return true;
}

bool Polynomials::startHook()
{
    /// set filter terms
    for (unsigned int i = 0; i < vector_size; i++)
    {
        /// Number of coefficients should be order+1
        if (orders[i]+1 != coefficients[i].size() )
        {
            log(Error)<<"Polynomial: order ("<<orders[i]<<") does not correspond with size number of coefficients ("<<coefficients[i].size()<<")"<<endlog();
            return false;
        }

        for (unsigned int j = 0; j < orders[i]+1; j++){
            polynomials[i].setTerm(j, coefficients[i][j]);
        }
    }


    // Check validity of Ports:
    if ( !inport.connected() ) {
        log(Error)<<"inputport not connected!"<<endlog();
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"Outputport not connected!"<<endlog();
    }

    if ( vector_size < 1 ) {
        log(Error)<<"Polynomials parameters not valid!"<<endlog();
        return false;
    }

    return true;
}

void Polynomials::updateHook()
{
	
    /// Read the input port
    inport.read( input );

    for (unsigned int i = 0; i < vector_size; i++)
    {
        output[i] = polynomials[i].evaluate( input[i] );
    }
    //log(Warning)<<"Polynomial output = "<<output[0]<<endlog();

    // Write the outputs
    outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::Polynomials)
