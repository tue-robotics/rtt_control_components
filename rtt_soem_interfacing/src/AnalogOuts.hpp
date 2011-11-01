#ifndef ANALOGOUTS_HPP
#define ANALOGOUTS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>


using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  typedef std::vector<double> doubles;

  class AnalogOuts
  : public RTT::TaskContext
    {
    private:

    OutputPort<soem_beckhoff_drivers::AnalogMsg> Analog_out_port;
    InputPort<doubles> wheels_port;
    InputPort<double> spindle_port;

    soem_beckhoff_drivers::AnalogMsg amsg;

    // Declaring output vector to write to the stack
    doubles values;
    doubles wheels;
    double spindle;

    // Specify maximum voltages:
    doubles max_volt;
    bool safe;
    
    // Global variables:
    doubles output;

    public:

    AnalogOuts(const string& name);
    ~AnalogOuts();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
