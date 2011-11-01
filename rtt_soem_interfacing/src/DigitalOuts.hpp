#ifndef DigitalOuts_HPP
#define DigitalOuts_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>


using namespace RTT;

namespace AMIGO // Just because it looks nice
{

  class DigitalOuts
  : public RTT::TaskContext
    {
    private:

    OutputPort<soem_beckhoff_drivers::DigitalMsg> digital_out_port;
    InputPort<bool> amplifiers_port;
    InputPort<bool> tuelights_port;
    InputPort<bool> spindlebrake_port;
    InputPort<bool> red_port;
    InputPort<bool> green_port;
    InputPort<bool> blue_port;
     

    soem_beckhoff_drivers::DigitalMsg dmsg;

    // Declaring output vector to write to the stack
    std::vector< bool > bits;
    bool amplifiers;
    bool tuelights;
    bool spindlebrake;
	// HACK
    long double start_time;
    // ENDHACK

    public:

    DigitalOuts(const string& name);
    ~DigitalOuts();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
