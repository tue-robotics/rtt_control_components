#ifndef THREADINGTESTCONTR_HPP
#define THREADINGTESTCONTR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created.

using namespace std;
using namespace RTT;

namespace THREADINGTESTCONTR
{
	
  class ThreadingTestContr
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    
	template <class T>
		inline string to_string (const T& t){
		stringstream ss;
		ss << t;
		return ss.str();
	};

    InputPort<doubles> inport;
    OutputPort<doubles> outport;
    
    public:
    
    // TaskContext pointer to component peer
	TaskContext* ThreadingTestRead;

    // Operations in Component Peers that homing component can call
	OperationCaller<void()> ResetEnc;

    ThreadingTestContr(const string& name);
    ~ThreadingTestContr();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
