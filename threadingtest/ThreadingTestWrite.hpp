#ifndef THREADINGTESTWRITE_HPP
#define THREADINGTESTWRITE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created.

using namespace std;
using namespace RTT;

namespace THREADINGTESTWRITE
{
	
  class ThreadingTestWrite
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

    ThreadingTestWrite(const string& name);
    ~ThreadingTestWrite();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
