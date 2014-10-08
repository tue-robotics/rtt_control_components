#ifndef REFERENCEGENERATOR_HPP
#define REFERENCEGENERATOR_HPP

#define maxN 10 //Maximum matrix size

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SOURCES
{
  typedef vector<double> doubles;
  typedef vector<int> ints;
  

  class ReferenceGenerator
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> posinport[5];
    InputPort<doubles> initialposinport;
    OutputPort<doubles> posoutport;
    OutputPort<doubles> veloutport;
    OutputPort<doubles> accoutport;
    OutputPort<sensor_msgs::JointState> resetrefoutport;

    // Properties
    uint N;
    uint N_inports;
    doubles minpos;
    doubles maxpos;
    doubles maxvel;
    doubles maxacc;
    ints inport_sizes;

    // Declaring global variables
    std::vector<refgen::RefGenerator> mRefGenerators;
    std::vector<amigo_msgs::ref_point> mRefPoints;
    doubles desiredPos;
    doubles desiredVel;
    doubles desiredAcc;
    doubles interpolators[maxN];
    double InterpolDt, InterpolEps;
    sensor_msgs::JointState resetRefMsg;

    public:

    ReferenceGenerator(const string& name);
    ~ReferenceGenerator();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
