/** Polynomials.hpp
 *
 * @class Polynomials
 *
 * \author Bas Willems
 * \date August, 2011
 * \version 1.0
 *
 */

#ifndef POLYNOMIALS_HPP
#define POLYNOMIALS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <math.h>

#define maxN 10 //Maximum matrix size. Still a workaround.

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};


using namespace RTT;

namespace MATH
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that computes the polynomial outcome for 
   * multiple polynomials
   *
   * The component has one input port that should receive an array.
   *
   * @param * polynomials - vector containing vectors of polynomials
   *        
   */
   
  class Polynomials
  : public RTT::TaskContext
    {
    private:

		/* Declaring input and output ports*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;
		
		// Array containing polynomials
		doubles polynomials[maxN];
		// Inputport size
		uint vector_size;
		
    public:

		Polynomials(const string& name);
		~Polynomials();

		bool configureHook();
		bool startHook();
		void updateHook();
		//double power(double x, int y);

    };
}
#endif
