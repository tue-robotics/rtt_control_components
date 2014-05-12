/** TracingOneSignal.cpp
 *
 * @class Tracing
 *
 * \author Tim Clephas
 * \date Summer, 2012
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <fstream>


#include "TracingOneSignal.hpp"

using namespace std;
using namespace RTT;
using namespace Signal;

TracingOneSignal::TracingOneSignal(const string& name) :
											TaskContext(name, PreOperational),
											filename("data.txt"),
											buffersize(16384),
											Ts(0.001)
{
	addProperty( "vector_sizes", vectorsizes_prop ).doc("size of the vector per port. Example: array ( 2.0 4.0 4.0 )");
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "filename", filename ).doc("Name of the file");
	addProperty( "Ts", Ts ).doc("Sample time of orocos, used for time vector");
}

TracingOneSignal::~TracingOneSignal(){}

bool TracingOneSignal::configureHook()
{
	columns = 0;

	// Create ports based on the number of vector sizes given
    Nports = vectorsizes_prop.size()+1;
	vectorsizes.resize(Nports);
	counters.resize(Nports);

    // Creating time slot
    vectorsizes[0] = 1.0;
    counters[0] = 0;
    columns = 1;


	// Creating ports
    for ( uint i = 1; i < Nports; i++ )
	{
		vectorsizes[i] = vectorsizes_prop[i-1]; // Hack
		counters[i] = 0;
        string name_inport = "in"+to_string(i);
		addEventPort( name_inport, inports[i-1]);
		columns += vectorsizes[i];
		cout << "Column size: ";
		cout << columns;
		cout << "\n";
	}

	counter = -1;

	buffers.resize(buffersize); // Maybe create reserve()?
	for (uint line = 0; line < buffersize; line++)
	{
		buffers[line].resize(columns,-1);
	}


	return true;
}

bool TracingOneSignal::startHook()
{
	/*if ( !inports[0].connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}*/
	current_ticks = 0;
    previous_ticks = RTT::os::TimeService::Instance()->getTicks();
	return true;
}

void TracingOneSignal::updateHook()
{
	// First updatehook is useless
	if(counter == -1){counter = 0;return;}
	// TODO: if ( !this->isRunning() ) return; (Check if this also works, counter can become a uint

	uint startcolumn = 0;
    for ( uint i = 1; i < Nports; i++ )
	{
        doubles input(vectorsizes[i],-2.0);

		if ( inports[i-1].read( input ) == NewData && counters[i] <= counter )
		{
            // Save time stamp
            if ( startcolumn == 0 )
            {
                buffers[counters[0]][startcolumn] = RTT::os::TimeService::Instance()->secondsSince(previous_ticks);
                startcolumn += vectorsizes[0];
                counters[0]++;
            }

			uint inputiterator = 0;
			// Fill it with data
			for (uint column =  startcolumn; column < startcolumn + vectorsizes[i]; ++column)
			{
				buffers[counters[i]][column] = input[inputiterator];
				inputiterator++;
			}

		}
        counters[i]++;
		startcolumn += vectorsizes[i];
	}

	counter = *max_element(counters.begin(),counters.end());

	// Stop if buffer is full
	if(abs(counter) >= buffersize) 
	{
		stop();
	}
}

void TracingOneSignal::stopHook()
{
	FILE * pFile;
	pFile = fopen (filename.c_str(),"w");


    fprintf(pFile, "SecondsPast\t");
	for ( uint i = 1; i < vectorsizes.size(); i++ )
	{
		string portname = inports[i-1].getName();
		for ( int j = 0; j < vectorsizes[i]; j++ )
		{
            fprintf(pFile, "%s[%d]\t", portname.c_str(), j );
			//char columnheader = "["+to_string(j)+"]\t";
			//portname.c_str()+
			//fprintf(pFile, "%s\t", columnheader);
			//cout << columnheader;
		}
	}
	fprintf(pFile, "\n");
	//

	uint line = 0;
	std::vector<doubles>::iterator vit;
	doubles::iterator dit;
	for(vit = buffers.begin(); vit != buffers.end(); ++vit)
	{
        //double timevalue = line*Ts;
        //fprintf(pFile, "%12.6e\t", timevalue);
		for(dit = (*vit).begin(); dit != (*vit).end(); ++dit)
		{
            fprintf(pFile, "%12.6e\t", *dit);
			//fprintf(pFile, "% f\t", *dit);
		}
		fprintf(pFile, "\n");
		line++;
	}


	fclose(pFile);
	
	cout << "Trace written!!!! Finished Tracing !!!!!!     End of Tracing Buffer reached, Controller can be terminated!" << endl;

}

ORO_CREATE_COMPONENT(Signal::TracingOneSignal)
