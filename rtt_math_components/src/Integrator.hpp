#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


//#define maxN 10 //Maximum matrix size. Still a workaround.


using namespace std;
using namespace RTT;

namespace MATH // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class Integrator
  : public RTT::TaskContext
    {
    private:

    // Declaring input and output ports
    InputPort<doubles> inport;
    InputPort<doubles> resetport;
    InputPort<doubles> initialport;
    OutputPort<doubles> outport;

    // Declaring global variables
    long double old_time;
    doubles previous_output;

    // Delacring variables set by properties
    uint N; // Number of ports to be calculated.

    public:

    Integrator(const string& name);
    ~Integrator();

    bool configureHook();
    bool startHook();
    void updateHook();
    double determineDt();
    };
}
#endif
