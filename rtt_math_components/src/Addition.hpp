/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Addition.hpp
 * Last modification:    March 2011
 */

#ifndef ADDITION_HPP
#define ADDITION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define maxN 10 //Maximum number of ports that can be created.

using namespace std;
using namespace RTT;

namespace MATH // Just because it looks nice
{
  /**
   * @brief A Component that adds two vectors elementwise.
   *
   * The component has two input ports that should receive vectors
   * of the same size. The first port is an eventport which will
   * trigger the component.
   *
   * @par Configuration
   * The component is configured using only the 'vectorsize'-property.
   */

  class Addition
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

    InputPort<doubles> inports[maxN];
    OutputPort<doubles> outport;

    uint vectorsize;
    uint numberofinputs;
    
    vector<doubles> inputs;

    public:

    Addition(const string& name);
    ~Addition();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
