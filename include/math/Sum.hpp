/** Sum.hpp
 *
 * @class Sum
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */
 
#ifndef SUM_HPP
#define SUM_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <sstream>

#define maxN 10 //Maximum number of ports that can be created.

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace MATH
{
	typedef vector<double> doubles;
	
  /**
   * @brief A Component that adds or substracs inputs
   * 
   * The component has as many input ports as number of listed signes
   * in the list_of_signs property. Each of the inputs is vector of 
   * vector_size
   * Input ports are eventports which will trigger the component.
   *
   * @param * list_of_signs - determines the number of input ports and 
   *        specifies the input sign for the each of them. It can only
   *        consist of pluses ("+") or minuses ("-"). 
   *        e.g. "+++-" or "+--".
   *        * vector_size - size of input vectors
   */
   
  class Sum
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inports[maxN];
		OutputPort<doubles> outport;

		uint N;
	
		/* Declaring variables set by properties */
		// List of signs
		std::string list_of_signs;
		uint vector_size;
		
    public:

		Sum(const string& name);
		~Sum();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
