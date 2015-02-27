/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gain.hpp
 * Last modification:    March 2011
 */

#ifndef SINE_HPP
#define SINE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace MATH
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

  class Sine
  : public RTT::TaskContext
    {
    private:

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
    Sine(const string& name);
    ~Sine();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    };
}
#endif
