#ifndef THREADINGTESTREAD_HPP
#define THREADINGTESTREAD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created.

using namespace std;
using namespace RTT;

namespace THREADINGTESTREAD
{
	
  class ThreadingTestRead
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
    
    doubles input;
    double reset_value;

    public:
    
	virtual void ResetEnc();


    ThreadingTestRead(const string& name);
    ~ThreadingTestRead();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
