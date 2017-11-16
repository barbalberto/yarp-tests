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
class FrameGrabberCrop_client : public RFModule
{
private:

    PolyDriver                      client;
    IFrameGrabberImage             *iFrameImage;
    VectorOf<std::pair<int, int> >  vertices;
    ImageOf<PixelRgb>               crop;
    Port                            outputPort;

public:

    FrameGrabberCrop_client()   { }

    double getPeriod()          { return 0.5; }

    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        Property config;

        config.put("device", "remote_grabber");
        config.put("no_stream", true);

        if(rf.check("remote"))
            config.put("remote", rf.find("remote").asString());
        else
            config.put("remote", "/server");

        if(rf.check("local"))
            config.put("local", rf.find("local").asString());
        else
            config.put("local", "/client");


        if(!client.open(config))
            return false;

        client.view(iFrameImage);

        if(!iFrameImage)
            return false;

        outputPort.open("/outputCrop" + rf.find("local").asString());
        yDebug() << "Config done!";
        return true;
    }

    bool updateModule()
    {
        vertices.clear();
        vertices.resize(2);
        vertices[0] = std::pair<int, int> (10, 10);
        vertices[1] = std::pair<int, int> (20, 30);

        iFrameImage->getImageCrop(YARP_CROP_RECT, vertices, crop);
        outputPort.write(crop);
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
    FrameGrabberCrop_client RFTest;
    RFTest.runModule(rf);
    return 0;
}


