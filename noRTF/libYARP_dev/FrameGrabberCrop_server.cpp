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
#include <ostream>      // std::cout, std::fixed
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision


#include <yarp/os/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/all.h>
#include <yarp/sig/all.h>


#include <float.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


/**************** Thread ***********************/
class FrameGrabberCrop_server : public RFModule
{
private:

    PolyDriver           server;
    IFrameGrabberImage  *iFrameImage;

public:

    FrameGrabberCrop_server()   { }

    double getPeriod()          {  return 0.5; }

    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        Property config;

        config.put("device",    "grabberDual");
        config.put("subdevice", "test_grabber");
//         config.put("split",     1);
        config.put("name",      "/server");
            config.put("period",     0.2);
        config.put("mode",      "grid");

        yInfo() << " server config options " << config.toString();
        if(!server.open(config))
            return false;

        yDebug() << "Config done!";
        return true;
    }

    bool updateModule()
    {
        yTrace();
        return true;
    }

    bool interruptModule()
    {
        yTrace();
        return true;
    }
};

/***********************************************/
int main(int argc, char *argv[])
{
    // initialize yarp with system clock
    yarp::os::Network yarp();
    if(!yarp::os::Network::checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.configure(argc, argv);
    FrameGrabberCrop_server RFTest;
    RFTest.runModule(rf);
    return 0;
}


