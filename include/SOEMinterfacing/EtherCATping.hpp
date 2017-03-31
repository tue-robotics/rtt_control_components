#ifndef EtherCATping_HPP
#define EtherCATping_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define N_PORTS 7 /* maximum number of ports */


using namespace std;

template <class T>
inline string to_string (const T& t){
	stringstream ss;
	ss << t;
	return ss.str();
};

using namespace RTT;

namespace ETHERCATPING
{
	class EtherCATping
	: public RTT::TaskContext
	{

	private:

	public:

	InputPort<soem_beckhoff_drivers::EncoderMsg> inports[N_PORTS];
	soem_beckhoff_drivers::EncoderMsg in_msg;
	vector<uint16_t> slave_cntrs;
	uint16_t cntr;
	uint countdown_timer;

	EtherCATping(const string& name);
	~EtherCATping();

	bool configureHook();
	bool startHook();
	void updateHook();
	void sendUpdate(uint n_times);
	};
}
#endif
