#ifndef SensorTorquesAutoC3_HPP
#define SensorTorquesAutoC3_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

using namespace std;
using namespace RTT;

namespace ARM
{
  typedef vector<double> doubles;
  typedef vector<int> ints;
  
  class SensorTorquesAutoC3
  : public RTT::TaskContext
    {
    private:

    InputPort<doubles> voltage_inport;
    OutputPort<doubles> measured_torques_outport;

	int cntr;
	bool EstimationComplete;
	bool EstimationStarted;
	
	unsigned int Nin;
    unsigned int Nout;
    ints obs_inputs;

	doubles c1;
	doubles c2;
	doubles c3;
	doubles Vmeasured;
	doubles Tmeasured;
	doubles Tmean;
	
    public:

    SensorTorquesAutoC3(const string& name);
    ~SensorTorquesAutoC3();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
