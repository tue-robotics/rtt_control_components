/** RTTPlot.cpp
*
* @class RTTPlot
*
* \author Janno Lunenburg
* \date December, 2015
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <QApplication>
//#include <QWidget>
//#include <qcustomplot.h>
//#include <QInputDialog>
//#include <QMessageBox>
//#include <QPushButton>
//#include <QDialogButtonBox>

#include "sinks/RTTPlot.hpp"
#include "sinks/RTTWidget.hpp"

using namespace RTT;
using namespace SINKS;

RTTPlot::RTTPlot(const std::string& name) :
	TaskContext(name, PreOperational)
{

//    QCustomPlot qcp;
//    qcp.show();

//    q_thread_ = new boost::thread(q_app_->exec);
    q_thread_ = new boost::thread( boost::bind( &RTTPlot::initQApp, this) );
//    q_app_->exec();

//    QApplication app(argc, argv);
//    QMessageBox mbox;
//    mbox.setText(QString::fromStdString("banana"));
//    mbox.exec();
}

RTTPlot::~RTTPlot()
{
    delete main_widget_;
    delete q_app_;
    q_thread_->join();
    delete q_thread_;
}

void RTTPlot::initQApp(){
    int argc = 0;
    char** argv;

    std::cout << "Starting qapp" << std::endl;
    q_app_ = new QApplication(argc, argv);

    std::cout << "Creating RTTPlotWidget" << std::endl;
    main_widget_ = new RTTPlotWidget;

    std::cout << "Showing main widget" << std::endl;
    main_widget_->show();

    std::cout << "Executing qapp" << std::endl;
    q_app_->exec();

}

bool RTTPlot::configureHook()
{
  
  Ts = getPeriod();
  
  return true;
}

bool RTTPlot::startHook()
{
  
  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }

  return true;
}

void RTTPlot::updateHook()
{

}

ORO_CREATE_COMPONENT(SINKS::RTTPlot)
