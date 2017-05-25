/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
class YarpClockProducer : public RFModule
{
private:
    int counter;
    yarp::os::BufferedPort<yarp::os::Bottle> port;

public:

    YarpClockProducer(bool system=true)
    {
        yTrace();
        counter = 0;
        port.open("/clock");
    }

    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        return true;
    }

    bool updateModule()
    {
        counter++;
        yarp::os::Bottle &b = port.prepare();
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
        return true;
    }
};

/***********************************************/
int main(int argc, char *argv[])
{    
    // initialize yarp without clock
    yarp::os::Network::initMinimum();
    if(!yarp::os::Network::checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.configure(argc, argv);
    YarpClockProducer RFTest;
    RFTest.runModule(rf);
    return 0;
}


