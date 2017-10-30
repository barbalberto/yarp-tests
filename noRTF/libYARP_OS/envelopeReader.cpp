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
class TimeStamp_Consumer : public RFModule
{
private:
    bool    success;
    Stamp   stamp;
    yarp::os::BufferedPort<yarp::os::Bottle> port;

public:

    TimeStamp_Consumer(bool system=true)
    {
        yTrace();
    }

    double getPeriod()
    {
        return 0.001;
    }

    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        port.open("/timeStamp_Consumer");
        while(!yarp::os::Network::connect("/timeStamp_Producer", "/timeStamp_Consumer", "udp") )
            yarp::os::Time::delay(0.05);
        return true;
    }

    bool updateModule()
    {
        Bottle *bRead = port.read();
        success = port.getEnvelope(stamp);
        yInfo("Success:  %d, read: %s, count: %d, tv: %0.30f", success, bRead->toString().c_str(), stamp.getCount(), stamp.getTime());
        return true;
    }

    bool interruptModule()
    {
       yTrace();
       port.interrupt();
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
    // initialize yarp with system clock
    yarp::os::Network yarp(/*YARP_CLOCK_SYSTEM*/);
    if(!yarp::os::Network::checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.configure(argc, argv);
    TimeStamp_Consumer RFTest;
    RFTest.runModule(rf);
    return 0;
}
