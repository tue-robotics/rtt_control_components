#ifndef SUBTRACTION_HPP
#define SUBTRACTION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace RTT;

// Define a new type for easy coding:
typedef vector<double> doubles;

class Subtraction
: public RTT::TaskContext
  {
  private:

  // Declaring input- and output_ports
  InputPort<doubles> inport_plus;
  InputPort<doubles> inport_minus;
  OutputPort<doubles> outport;


  // Declaring global variables

  // Delacring variables set by properties

  uint vectorsize; // Number of errors to be calculated.

  public:

  Subtraction(const string& name);
  ~Subtraction();

  bool configureHook();
  bool startHook();
  void updateHook();
  };
#endif
