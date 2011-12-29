/** Reporter.hpp
 *
 * @class Reporter
 *
 * \author Tim Clephas
 * \date December, 2011
 * \version 1.0
 *
 */

#ifndef Reporter_HPP
#define Reporter_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <rtt/os/TimeService.hpp>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace CUSTOM
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

    /**
   * @brief A Component that concatenates input vectors into one
   *        output vector
   *
   * The component has N input ports that should receive vectors of the
   * same size. It outputs one vector of the length that equals the
   * sum of the lengths of input vectors
   *
   * @param N [2] - number of input ports (vectors)
   */

  class Reporter
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inports[maxN];

		/* Declaring variables set by properties */
		// Number of input ports
		uint N;
		string filename;
		string firstline;

		// Declaring global variables
		ofstream file;
		RTT::os::TimeService::ticks starttime;
		RTT::os::TimeService::Seconds timestamp;

    public:

		Reporter(const string& name);
		~Reporter();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();

    };
}
#endif
