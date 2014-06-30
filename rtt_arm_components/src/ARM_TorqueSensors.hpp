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
  typedef vector<int> ints;
  
  class SensorTorques
  : public RTT::TaskContext
    {
    private:

    InputPort<doubles> voltage_inport;
    OutputPort<doubles> measured_torques_outport;

    unsigned int Nin;
    unsigned int Nout;
    ints obs_inputs;

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
