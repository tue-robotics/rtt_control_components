/** RTTPLOT.hpp
 *
 * @class ConstantBool
 *
 * \author Janno Lunenburg
 * \date December, 2015
 * \version 1.0
 *
 */

#ifndef RTTPLOT_HPP
#define RTTPLOT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <boost/thread.hpp>

using namespace RTT;

class QApplication;

namespace SINKS
{

/**
   * @brief A Component to visualize RTT signals in realtime
   */

class RTTPlot
        : public RTT::TaskContext
{
public:

    RTTPlot(const std::string& name);
    ~RTTPlot();

    bool configureHook();
    bool startHook();
    void updateHook();

private:

    double Ts;

    QApplication* q_app_;

    boost::thread* q_thread_;



};
}
#endif
