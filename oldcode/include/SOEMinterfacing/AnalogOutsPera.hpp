#ifndef ANALOGOUTSPERA_HPP
#define ANALOGOUTSPERA_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>


using namespace std;
using namespace RTT;

namespace SOEM
{
  typedef std::vector<double> doubles;

  class AnalogOutsPera
  : public RTT::TaskContext
    {
    private:

    OutputPort<soem_beckhoff_drivers::AnalogMsg> out_port1;
    OutputPort<soem_beckhoff_drivers::AnalogMsg> out_port2;
    OutputPort<soem_beckhoff_drivers::AnalogMsg> out_port3;
    InputPort<doubles> rpera_port;

    soem_beckhoff_drivers::AnalogMsg amsg1;
    soem_beckhoff_drivers::AnalogMsg amsg2;
    soem_beckhoff_drivers::AnalogMsg amsg3;

    // Declaring output vector to write to the slaves
    doubles rpera;
    doubles output1;
    doubles output2;
    doubles output3;

	public:

    AnalogOutsPera(const string& name);
    ~AnalogOutsPera();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
