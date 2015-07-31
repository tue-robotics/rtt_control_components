/** DoublesToROS.cpp
 *
 * @class DoublesToROS
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 2.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "DoublesToROS.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

DoublesToROS::DoublesToROS(const string& name) :
    TaskContext(name, PreOperational)
{
    // Adding ports
    Ndouble = 1;
    addPort( "in", doubleinport );
    addProperty( "NumberOfDoublesInVector", Ndouble );

    sendAnArray = false;
    addProperty("sendAnArray",sendAnArray).doc("if true, input will be mapped to a ros multiarray message and send to a single output");
}

DoublesToROS::~DoublesToROS(){}

bool DoublesToROS::configureHook()
{
    if(Ndouble > maxN)
        log(Error) << "doublesToRos assumes max input length of " << maxN << "!" << endlog();

    if( sendAnArray )
        addPort("out1", arrayoutport);
    else
    {
        for ( uint i = 0; i < Ndouble; i++ ) {
            string name_outport = "out"+to_string(i+1);
            addPort( name_outport, doubleoutports[i] );
        }
    }

    return true;
}

bool DoublesToROS::startHook()
{
    return true;
}

void DoublesToROS::updateHook()
{

    doubles values;
    if ( doubleinport.read( values ) == NewData ) {
        if( sendAnArray )
        {   
            std_msgs::MultiArrayDimension dim;
            dim.size = Ndouble;
            dim.label = "Out1_DoublesToRos";

            std_msgs::Float32MultiArray msg;
            msg.layout.dim.push_back(dim);

            for(uint i = 0; i < Ndouble; ++i)
                msg.data.push_back(values[i]);

            arrayoutport.write(msg);

        }
        else
        {
            for ( uint i = 0; i < Ndouble; i++ )
            {
                std_msgs::Float32 msg;
                msg.data = values[i];
                doubleoutports[i].write( msg );
            }
        }
    }
}

ORO_CREATE_COMPONENT(ROS::DoublesToROS)
