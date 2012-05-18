/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gains.hpp
 * Last modification:    March 2011
 */

#ifndef GAINS_HPP
#define GAINS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
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

  class Gains
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

    /*
     * Declaring variable vectorsize which is set by a property
     */
    uint vectorsize; // Number of elements to be calculated.
    doubles gain;

    public:
    /**
     * Set up a component for adding two vectors.
     */
    Gains(const string& name);
    ~Gains();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    };
}
#endif