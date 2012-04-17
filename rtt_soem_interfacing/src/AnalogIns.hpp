#ifndef AnalogIns_HPP
#define AnalogIns_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <std_msgs/Float32.h>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SOEM
{
  class AnalogIns
  : public RTT::TaskContext
    {
	
    private:

    InputPort<soem_beckhoff_drivers::AnalogMsg> inport;
    OutputPort<std_msgs::Float32> outport[2];
	
    public:

    AnalogIns(const string& name);
    ~AnalogIns();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
