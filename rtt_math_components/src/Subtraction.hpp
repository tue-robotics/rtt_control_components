/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Subtraction.hpp
 * Last modification:    March 2011
 */

#ifndef SUBTRACTION_HPP
#define SUBTRACTION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace RTT;

namespace MATH // Just because it looks nice
{
  /**
   * @brief A Component that subtracts two vectors elementwise.
   *
   * The component has two input ports that should receive vectors
   * of the same size. The minus port is an eventport which will
   * trigger the component.
   *
   * @par Configuration
   * The component is configured using only the 'vectorsize'-property.
   */

  class Subtraction
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
  InputPort<doubles> inport_plus;
  InputPort<doubles> inport_minus;
  OutputPort<doubles> outport;

    /*
     * Declaring variable vectorsize which is set by a property
     */
    uint vectorsize; // Number of elements to be calculated.

    public:
    /**
     * Set up a component for subtracting two vectors.
     */
    Subtraction(const string& name);
    ~Subtraction();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
