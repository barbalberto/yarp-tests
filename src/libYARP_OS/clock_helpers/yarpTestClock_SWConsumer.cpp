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
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


#define DEFAULT_THREAD_REATE_MS 1000

/**************** Thread ***********************/
class YarpClock_SWConsumer_Thread : public RateThread
{
public:
    YarpClock_SWConsumer_Thread() : RateThread(DEFAULT_THREAD_REATE_MS)
    {
        yTrace();
    }

    bool threadInit()
    {
        yTrace() << "YarpClock_SWConsumer_Module " << yarp::os::Time::now();
        return true;
    }

    void run()
    {
        yInfo() << "Thread " << yarp::os::Time::now();
    }

    void threadRelease()
    {
        yTrace() << yarp::os::Time::now();
    }
};

/**************** RFModule ***********************/
class YarpClock_SWConsumer_Module : public RFModule
{
public:
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    YarpClock_SWConsumer_Module(bool system=true)
    {
        yTrace();
//        counter = 0;
//        port.open("/clock");
//        yarp::os::Time::useSystemClock();
//        yInfo() << "is SystemClock? " << yarp::os::Time::isSystemClock();
    }
    int counter;
    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        return true;
    }

    bool updateModule()
    {
        yDebug() << "Module " << yarp::os::Time::now();

//        yDebug() << yarp::os::SystemClock::nowSystem();
//        counter++;
//        yarp::os::Bottle &b = port.prepare();
//        double ciao = yarp::os::Time::now();
//        b.clear();
//        b.addInt(floor(counter));
//        b.addInt(0);
//        port.write();
//        yTrace() << ":  " << counter;
        return true;
    }

    bool interruptModule()
    {
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

    Port    feedbackPort;
    Bottle  status;
    Stamp   stamp;

    yInfo() << "\t Opening ports";
    feedbackPort.open("/feedback_SW_cons_1");
    yarp::os::Network::connect("/feedback_SW_cons_1", "/feedback");

    status.addString("started");
    feedbackPort.write(status);

    int delay = 4;
    yInfo() << "\n\t Waiting " << delay << " seconds";
    yarp::os::SystemClock::delaySystem(delay);

    yInfo() << "\n\t Creating network clock";
    yarp::os::Time::useNetworkClock("/clock");

    yInfo() << "\n\t After check network Clock, before creating rateThread()";
    YarpClock_SWConsumer_Thread thread;

    yInfo() << "\n\t RateThread instantiated (not started yet)";
    thread.start();

    yInfo() << "\n\t RateThread started";

    ResourceFinder rf;
    yInfo() << "\n\t ResourceFinder instantiated";

    rf.configure(argc, argv);
    yInfo() << "\n\t ResourceFinder configured";

    YarpClock_SWConsumer_Module RFTest;
    yInfo() << "\n\t RFModule instantiated";

    yInfo() << "\n\t before calling RFModule.runModule()";
    int ret = RFTest.runModule(rf);
    yInfo() << "\n\t RFModule returned with value " << ret;
    return 0;
}


