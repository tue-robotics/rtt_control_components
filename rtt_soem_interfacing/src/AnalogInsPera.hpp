#ifndef AnalogInsPera_HPP
#define AnalogInsPera_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>

using namespace std;

using namespace RTT;

namespace SOEM
{
  typedef std::vector<double> doubles;

  class AnalogInsPera
  : public RTT::TaskContext
    {
	
    private:

    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_for1;
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_for2;
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_for3;
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_pos1;
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_pos2;
    InputPort<soem_beckhoff_drivers::AnalogMsg> inport_pos3;

    OutputPort<doubles> outport_for;
    OutputPort<doubles> outport_pos;

    soem_beckhoff_drivers::AnalogMsg amsgf1;
    soem_beckhoff_drivers::AnalogMsg amsgf2;
    soem_beckhoff_drivers::AnalogMsg amsgf3;
    soem_beckhoff_drivers::AnalogMsg amsgp1;
    soem_beckhoff_drivers::AnalogMsg amsgp2;
    soem_beckhoff_drivers::AnalogMsg amsgp3;

    doubles output_for;
    doubles output_pos;
	
    public:

    AnalogInsPera(const string& name);
    ~AnalogInsPera();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
