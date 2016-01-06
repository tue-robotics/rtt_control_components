#ifndef THREADINGTESTBASE_HPP
#define THREADINGTESTBASE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created.

using namespace std;
using namespace RTT;

namespace THREADINGTESTBASE
{
	
  class ThreadingTestBase
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
    
    string compname;

    public:

    ThreadingTestBase(const string& name);
    ~ThreadingTestBase();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
