/** RealTimeDerivator.hpp
 *
 * @class RealTimeDerivator
 *
 * \author Sava Marinkov
 * \date May, 2011
 * \version 1.0
 *
 */

#ifndef REALTIMEDERIVATOR_HPP
#define REALTIMEDERIVATOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#define MAX_SIZE	20
#define PI 			3.14159265358979

using namespace std;
using namespace RTT;

namespace AMIGO
{
  typedef vector<double> doubles;
  
  class RealTimeDerivator
  : public RTT::TaskContext
    {
    private:
		double determineDt();
				
		InputPort<doubles> inport;
		OutputPort<doubles> ue_port;
		OutputPort<doubles> ude_port;
		OutputPort<doubles> udde_port;
		
		OutputPort<std_msgs::Float32MultiArray> msg_port;
		
		double Ts;
		long double old_time;
		int vector_size;
		
		double ue_prev[MAX_SIZE];
		double ude_prev[MAX_SIZE];
		double udde_prev[MAX_SIZE];
		
		double f, k1, k2;
		
    public:

		RealTimeDerivator(const string& name);
		~RealTimeDerivator();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
