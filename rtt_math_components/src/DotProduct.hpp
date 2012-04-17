/** DotProduct.hpp
*
* @class DotProduct
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#ifndef DOTPRODUCT_HPP
#define DOTPRODUCT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace std;
using namespace RTT;

namespace MATH
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  /**
   * @brief A Component that executes product of N vectors elementwise.
   *
   * The component has as many input ports as number of listed operators
   * in the list_of_operators property. Each of the inputs is vector of 
   * vector_size.
   * Input ports are eventports which will trigger the component.
   *
   * @param list_of_operators - determines the number of input ports and 
   *        specifies the input operator for the each of them. It can 
   *        only consist of operators multiply ("*") or divide ("/"),
   *        e.g. "*//**" or "/**".
   *        * vector_size - size of input vectors
   */

  class DotProduct
  : public RTT::TaskContext
    {
    private:

		/*** Declaring input and output ports ***/
		InputPort<doubles> inports[maxN];
		OutputPort<doubles> outport;

		/*** Declaring global variables ***/
		uint N;		

		/*** Declaring number of inputs which is set by a property ***/
		// List of operators
		std::string list_of_operators;
		uint vector_size;

    public:
    
		/*** Set up a component for adding two vectors ***/
		DotProduct(const string& name);
		~DotProduct();

		bool configureHook();
		bool startHook();
		void updateHook();
    };
}

#endif
