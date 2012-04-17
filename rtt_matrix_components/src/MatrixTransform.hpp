#ifndef MATRIXTRANSFORM_HPP
#define MATRIXTRANSFORM_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


#define maxN 10 //Maximum matrix size. Still a workaround.

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class MatrixTransform
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> inport;
    OutputPort<doubles> outport;

    // Declare a PropertyBag containing the matrix.
    PropertyBag TransformationMatrix;
    uint Nrows;
    uint Ncolumns;

    // Declaring global variables
    vector<doubles> matrix;
    doubles function[maxN];


    public:

    MatrixTransform(const string& name);
    ~MatrixTransform();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif

