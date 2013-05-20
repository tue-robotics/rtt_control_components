#ifndef READENCODERS_HPP
#define READENCODERS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <soem_beckhoff_drivers/EncoderMsg.h>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};


#define maxN 10 //Maximum number of ports that can be created. Still a workaround.


using namespace RTT;
using namespace soem_beckhoff_drivers;

namespace SOEM // Just because it looks nice
{
  typedef vector<double> doubles;

  class ReadEncoders
  : public RTT::TaskContext
    {
    private:
    // Number of encoders to read
    uint N;

    // Counter
    uint counter;

    // Declaring input- and output_ports
    InputPort<EncoderMsg> inport_enc[maxN];
    InputPort<bool> inport_reNull;
    OutputPort<doubles> outport;
    OutputPort<doubles> outport_enc;
    

    // Declaring message types
    double previous_enc_position[maxN];
    doubles SI_value;
    doubles ENC_value;
    doubles init_SI_value;
    int ienc[maxN];
    doubles enc2SI;
    doubles offset;
    uint encoderbits;
    

    public:

    ReadEncoders(const string& name);
    ~ReadEncoders();

    bool configureHook();
    bool startHook();
    void updateHook();
    void reset( uint Nreset, double resetvalue );

    private:

    double readEncoder( int i );
    };
}
#endif
