/**
 * author: Bas Willems
 * email:  b.willems@student.tue.nl
 *
 * filename:             AddConstant.hpp
 * Last modification:    March 2011
 */

#ifndef ADDCONSTANT_HPP
#define ADDCONSTANT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
using namespace RTT;

namespace MATH
{
  /**
   * @brief A Component adds a constant vector to the incoming vector 
   * elementwise.
   *
   * The component has one input port that should receive a vector
   * The inport is an eventport which will trigger the component.
   *
   * @par Configuration
   * The component is configured using the 'vectorsize'-property to set
   * the size of the incoming vector and the 'additions'-property being
   * a vector containing the values to be added elementwise.
   */

  class AddConstant
  : public RTT::TaskContext
    {
    private:

    /**
     * Define a new type doubles for easy coding.
     */
    typedef vector<double> doubles;


    /*
     * Declaring input- and output_ports
     */
    InputPort<doubles> inport;
    OutputPort<doubles> outport;
    
    doubles addconstants;

    /*
     * Declaring variable vectorsize which is set by a property
     */
    uint vectorsize; // Number of elements to be calculated.

    public:
    /**
     * Set up a component for adding a constant vector.
     */
    AddConstant(const string& name);
    ~AddConstant();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
