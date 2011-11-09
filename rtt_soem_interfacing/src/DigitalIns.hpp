#ifndef DigitalIns_HPP
#define DigitalIns_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <std_msgs/Bool.h>

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SOEM
{
  class DigitalIns
  : public RTT::TaskContext
    {
	
    private:

    InputPort<soem_beckhoff_drivers::DigitalMsg> inport;
    OutputPort<std_msgs::Bool> outport[8];
	
    public:

    DigitalIns(const string& name);
    ~DigitalIns();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
