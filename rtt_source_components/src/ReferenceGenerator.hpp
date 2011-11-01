#ifndef REFERENCEGENERATOR_HPP
#define REFERENCEGENERATOR_HPP

#define maxN 10 //Maximum matrix size. Still a workaround.

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

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

  class ReferenceGenerator
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> posinport;
    InputPort<doubles> resetPort;
    OutputPort<doubles> posoutport;
    OutputPort<doubles> veloutport;
    OutputPort<doubles> accoutport;

    uint NrInterpolators;

    // Declaring global variables
    std::vector<refgen::RefGenerator> mRefGenerators;
    std::vector<amigo_msgs::ref_point> mRefPoints;
    doubles desiredPos;
    doubles interpolators[maxN];
    double InterpolDt, InterpolEps;

    public:

    ReferenceGenerator(const string& name);
    ~ReferenceGenerator();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif