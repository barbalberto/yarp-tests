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
class YarpClockProducer : public RFModule
{
private:
    int     precision;
    int     counter;
    double  timeValue;
    Stamp   stamp;
    bool    test2;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    yarp::os::  Port portSimple;

public:

    YarpClockProducer(bool system=true)
    {
        yTrace();
        test2 = false;
        counter = 0;
        port.open("/timeStamp_Producer");
//         portSimple.open("/envelopeTest_1");
    }

    double getPeriod()
    {
        return 0.05;
    }

    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        counter   = 0;
        precision = 100000;
        timeValue = (double)counter / precision;
        yWarning(":  %d  tv: %0.10f", counter, timeValue);
        yarp::os::Time::delay(1);

        yInfo() << "FLT_DIG is     " << FLT_DIG;
        yInfo() << "DBL_DIG is     " << DBL_DIG;
        yInfo() << "DBL_MAX_EXP is " << DBL_MAX_EXP;
        yInfo() << "DBL_MAX is     " << DBL_MAX;

//         yInfo("Count:          %%(DBL_DIG)*e                         %%.*(DBL_DIG)g                    %%*.20g                      %%*.(DBL_DIG)f \n");

        return true;
    }


    bool sendEnvelope()
    {
        Bottle envelope;
        envelope.clear();
        envelope.addInt(counter);
        envelope.addString("SET_ENVELOPE");

        Bottle &data = port.prepare();
        data.clear();
        data.addString("This is the data");

        port.setEnvelope(envelope);
        port.write();
    }

    bool updateModule()
    {

        sendEnvelope();
        counter++;
        return true;

        Bottle envelope;
        envelope.clear();
        envelope.addInt(counter);
        envelope.addString("Envelope");

        Bottle &b = port.prepare();
        b.clear();
        b.addString("This is the data");

        if(!test2)
        {
            port.setEnvelope(envelope);
            port.write();

            yInfo("-------------------------------------------------");
            yInfo("%%f %-25f \t\t%%*f %*f \t\t%%*f %.*f", timeValue, DBL_DIG, timeValue, DBL_DIG, timeValue);
            yInfo("%%e %-25e \t\t%%*e %*e \t\t%%*e %.*e", timeValue, DBL_DIG, timeValue, DBL_DIG, timeValue);
            yInfo("%%g %-25g \t\t%%*g %*g \t\t%%*g %.*g", timeValue, DBL_DIG, timeValue, DBL_DIG, timeValue);
            yInfo("%d:   %e   %30.*g   %30.*g   %30.*f ", counter, timeValue, DBL_DIG, timeValue, 20, timeValue, DBL_DIG, timeValue);

            if(counter > 30)
            {
                test2       = true;
                counter     = 0;
                timeValue   = 10e55;
            }
        }
        else
        {
            b.addInt(counter);
            timeValue /=10;
            Bottle b;
            b.addString("ENVELOPE");
            stamp.update(timeValue);
            port.setEnvelope(b);
            port.write();
            if(counter > 30)
            {
                test2       = false;
                counter     = 0;
                timeValue   = 10e55;
            }
//             yInfo("%d:   %e  %.*e   %#.*g   %30.*g   %30.*f ", counter, timeValue, DBL_DIG, timeValue, DBL_DIG, timeValue, 20, timeValue, DBL_DIG, timeValue);
        }


//         Port portSimple
//         Bottle b2;
//         b2.addInt(counter);
//         portSimple.setEnvelope(stamp);
//         portSimple.write(b2);

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
    yarp::os::Network yarp(/*YARP_CLOCK_SYSTEM*/);
    if(!yarp::os::Network::checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.configure(argc, argv);
    YarpClockProducer RFTest;
    RFTest.runModule(rf);
    return 0;
}


