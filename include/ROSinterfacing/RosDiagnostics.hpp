/** RosDiagnostics.hpp
 *
 * @class RosDiagnostics
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 2.0
 *
 */

#ifndef ROSDIAGNOSTICS_HPP
#define ROSDIAGNOSTICS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#define maxN 40 //Maximum  size

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace ROS
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class RosDiagnostics
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<string> strings;

    /* Declaring and output port*/
    OutputPort<diagnostic_msgs::DiagnosticArray> diagnosticsport;

    InputPort<doubles> vectorports[maxN];
    InputPort<bool> boolports[maxN];

    /* Declaring property variables */
    string vectornames[maxN];
    string boolnames[maxN];

    /* Declaring global variables */
    uint Nvec; // Number of vectors to diagnose
    uint Nbool; // Number of booleans to diagnose
    string statusname;


    public:

    RosDiagnostics(const string& name);
    ~RosDiagnostics();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
