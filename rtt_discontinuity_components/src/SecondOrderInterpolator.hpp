#ifndef SECONDORDERINTERPOLATOR_HPP
#define SECONDORDERINTERPOLATOR_HPP

#define maxN 10 //Maximum matrix size. Still a workaround.

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace AMIGO
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class SecondOrderInterpolator
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> accinport;
    InputPort<doubles> velinport;
    InputPort<doubles> posinport;
    InputPort<doubles> resetPort;
    OutputPort<doubles> posoutport;
    OutputPort<doubles> veloutport;
    OutputPort<doubles> accoutport;

    uint NrInterpolators;

    // Declaring global variables
    std::vector<refgen::RefGenerator> mRefGenerators;
    std::vector<amigo_msgs::ref_point> mRefPoints;
    doubles desiredAcc;
    doubles desiredVel;
    doubles desiredPos;
    doubles interpolators[maxN];
    double InterpolDt, InterpolEps;

    public:

    SecondOrderInterpolator(const string& name);
    ~SecondOrderInterpolator();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
