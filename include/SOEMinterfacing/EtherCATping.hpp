#ifndef EtherCATping_HPP
#define EtherCATping_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/EncoderMsg.h>

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

	InputPort<soem_beckhoff_drivers::EncoderMsg> inports[6];
	soem_beckhoff_drivers::EncoderMsg in_msg;
	vector<uint16_t> slave_cntrs;
	uint16_t cntr;

	EtherCATping(const string& name);
	~EtherCATping();

	bool configureHook();
	bool startHook();
	void updateHook();
	};
}
#endif
