/**
*******************************************************************************
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*******************************************************************************
*/

#include <fstream>
#include <string>
#include <cstdio>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


/**************** Thread ***********************/
class ClockTestModule : public RFModule
{
    public:
        yarp::os::BufferedPort<yarp::os::Bottle> port;
        ClockTestModule()
        {
            yTrace();
            counter = 0;
            port.open("/clock");
            yarp::os::Time::useSystemClock();
            yInfo() << "is SystemClock? " << yarp::os::Time::isSystemClock();
        }
        int counter;
        bool configure(yarp::os::ResourceFinder &rf)
        {
            yTrace();
        }

        bool updateModule()
        {
            yDebug() << yarp::os::SystemClock::nowSystem();
            counter++;
            yarp::os::Bottle &b = port.prepare();
            double ciao = yarp::os::Time::now();
            b.clear();
            b.addInt(floor(counter));
            b.addInt(0);
            port.write();
            yTrace() << ":  " << counter;
            return true;
        }
        
        bool interruptModule()
        {
            yTrace();
            yarp::os::Time::useSystemClock();
            if(yarp::os::Time::isSystemClock())
            {
                yInfo() << "Current clock is SystemClock as expected";
                return true;
            }
            else
            {
                yError() << "Current clock is NOT SystemClock as it should be!!";
                return false;
            }
        }
};

/***********************************************/
int main(int argc, char *argv[])
{    
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ClockTestModule RFTest;
    RFTest.runModule();
    return 0;
}


