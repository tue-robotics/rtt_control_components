#ifndef PRINTLOG_HPP
#define PRINTLOG_HPP

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

using namespace std;
using namespace RTT;

class TueLog {
	public:
		void print(int logLevel, string componentname, string message) {
			Logger::In in(componentname); // To do: Can you automatically include the class as string here?
			if (logLevel == 0) log(Error)<< message <<endlog();
			if (logLevel == 1) log(Warning)<< message <<endlog();
			if (logLevel == 2) log(Info)<< message <<endlog();
			if (logLevel == 3) log(Debug)<< message <<endlog();
		};

	protected:
};

#endif
