#ifndef DigitalOuts_HPP
#define DigitalOuts_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>


using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace SOEM // Just because it looks nice
{

  class DigitalOuts
  : public RTT::TaskContext
    {
    private:

    OutputPort<soem_beckhoff_drivers::DigitalMsg> digital_out_port;
    //InputPort<bool> amplifiers_port;
    //InputPort<bool> tuelights_port;
    //InputPort<bool> spindlebrake_port;
    //InputPort<bool> red_port;
    //InputPort<bool> green_port;
    //InputPort<bool> blue_port;
    InputPort<bool> inport[8];
     
    soem_beckhoff_drivers::DigitalMsg dmsg;

    // Declaring output vector to write to the stack
    uint n_bits;
    //std::vector<bool> bits;
    //bool amplifiers;
    //bool tuelights;
    //bool spindlebrake;

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
