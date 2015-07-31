#ifndef SensorTorques_HPP
#define SensorTorques_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

using namespace std;
using namespace RTT;

namespace ARM
{
  typedef vector<double> doubles;
  
  class SensorTorques
  : public RTT::TaskContext
    {
    private:

    InputPort<doubles> voltage_inport;
    OutputPort<doubles> measured_torques_outport;

    unsigned int N;

	doubles c1;
	doubles c2;
	doubles c3;
	doubles Vmeasured;
	doubles Tmeasured;

    public:

    SensorTorques(const string& name);
    ~SensorTorques();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
