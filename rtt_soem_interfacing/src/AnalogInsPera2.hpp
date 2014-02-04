#ifndef AnalogInsPera_HPP
#define AnalogInsPera_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>

#define maxN 10 //Maximum number of PERA boards that is supported that can be created. Still a workaround.

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace SOEM
{
  typedef std::vector<double> doubles;

  class AnalogInsPera2
  : public RTT::TaskContext
    {
	
    private:

    // declaring of ports
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_for[maxN];
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_pos[maxN];
    OutputPort<doubles> outport_for;
    OutputPort<doubles> outport_pos;

    // declaring of msgs
    soem_beckhoff_drivers::AnalogMsg amsgf;
    soem_beckhoff_drivers::AnalogMsg amsgp;
    
    /** Actual number of connected I/O boards */
    unsigned int N;

    // declaring of output doubles
    doubles output_for;
    doubles output_pos;
	
    public:

    AnalogInsPera2(const string& name);
    ~AnalogInsPera2();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
