/** Product.hpp
*
* @class Product
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#ifndef PRODUCT_HPP
#define PRODUCT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
using namespace RTT;

namespace MATH
{
    /*** Define a new type doubles for easy coding ***/
    typedef vector<double> doubles;

  /**
   * @brief A Component that executes product of two vectors elementwise.
   *
   * The component has two input ports that should receive vectors
   * of the same size. The first port is an eventport which will
   * trigger the component.
   *
   * @param list_of_operators - determines the number of input ports and 
	        specifies the input operator for the each of them. It can only
	        consist of operators multiply ("*") or divide ("/"). 
	        e.g. "*//**" or "/**".
   */

  class Product
  : public RTT::TaskContext
    {
    private:

		/*** Declaring input and output ports ***/
		vector< InputPort<double> >inports;
		OutputPort<double> outport;

		/*** Declaring global variables ***/
		double Ts;
		uint N;		

		/*** Declaring number of inputs which is set by a property ***/
		// List of operators
		std::string list_of_operators;

    public:
    
		/*** Set up a component for adding two vectors ***/
		Product(const string& name);
		~Product();

		bool configureHook();
		bool startHook();
		void updateHook();
    };
}

#endif
