/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Addition.hpp
 * Last modification:    March 2011
 */

#ifndef ADDITION_HPP
#define ADDITION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace RTT;

namespace MATH // Just because it looks nice
{
  /**
   * @brief A Component that adds two vectors elementwise.
   *
   * The component has two input ports that should receive vectors
   * of the same size. The first port is an eventport which will
   * trigger the component.
   *
   * @par Configuration
   * The component is configured using only the 'vectorsize'-property.
   */

  class Addition
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
    InputPort<doubles> inport1;
    InputPort<doubles> inport2;
    OutputPort<doubles> outport;

    /*
     * Declaring variable vectorsize which is set by a property
     */
    uint vectorsize; // Number of elements to be calculated.

    public:
    /**
     * Set up a component for adding two vectors.
     */
    Addition(const string& name);
    ~Addition();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
