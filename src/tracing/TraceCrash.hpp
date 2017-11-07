/** TraceCrash.hpp
 *
 * @class TraceCrash
 *
 * \author Tim Clephas
 * \date Summer, 2012
 * \version 1.0
 *
 */

#ifndef TraceCrash_HPP
#define TraceCrash_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include "boost/multi_array.hpp"
#include <cassert>

#define maxN 10 //Maximum number of ports that can be created.


using namespace std;
using namespace RTT;

namespace Signal
{
    typedef vector<double> doubles;
    typedef vector<uint> uints;

    template <class T>
    inline string to_string (const T& t){
        stringstream ss;
        ss << t;
        return ss.str();
    };

    /**
       * @brief A Component that traces the last x data points of multiple signals.
       *
       * The component has input port as stated in property number_of_inputs,
       * that should receive vector of doubles.
       * Input ports are eventports which will trigger the component.
       *
       * @param * N - number of inputs
       *        * vector_size - size of input vectors
       */

    class TraceCrash
            : public RTT::TaskContext
    {
    private:

        /* Declaring and output port*/
        InputPort<doubles> inports[maxN];

        string filename;
        doubles vectorsizes_prop;
        vector<int> vectorsizes;
        uint buffersize;
        double Ts;

        doubles buffer;
        vector<doubles> buffers;
        int counter;
        uint columns;
        uint rows;
        uint Nports;
        bool printed;

    public:

        TraceCrash(const string& name);
        ~TraceCrash();

        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();

    };
}
#endif
