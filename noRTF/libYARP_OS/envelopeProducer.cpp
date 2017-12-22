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
#include <yarp/os/PortablePair.h>

#include <yarp/os/impl/PortCommand.h>  // funzionera?? Risposta: no!

#include <yarp/dev/all.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/IJoypadController.h>


#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Vec2D.h>
#include <yarp/math/Quaternion.h>


#include <float.h>

#include "portablesBestOf.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/**************** Thread ***********************/
class YarpClockProducer : public RFModule
{
private:
    int     counter;
    int     numberOfRun;
/*
    int     precision;
    double  timeValue;
*/
    bool    test2;
    bool    plain;
    bool    quiet;
    bool    verbose;
    Bottle  data, synchMsg;

    Port    synchPort;
    Port    portSimple;
    BufferedPort<Bottle> portBuff;

    PortablesBestOf  envelopes;

public:

    YarpClockProducer(bool system=true) : quiet(true), verbose(false)
    {
        yTrace();

        test2 = false;
        plain = true;
        counter = 0;
        data.clear();
        numberOfRun = 10;
    }

    double getPeriod()
    {
        return 0.001;
    }


    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
/*
        counter   = 0;
        precision = 100000;
        timeValue = (double)counter / precision;
*/
        synchPort.open(envProducer_SynchPort_name);

        if(rf.check("buff"))
            plain = false;

        if(rf.check("quiet"))
            quiet = false;

        if(rf.check("verbose"))
            verbose = false;

        if(plain)
            portSimple.open(envProducer_DataPort_name);
        else
            portBuff.open(envProducer_DataPort_name);

        envelopes.size = rf.check("size", Value(10)).asInt();
        numberOfRun    = rf.check("run",  Value(10)).asInt();

        yInfo() << "Running with size " << envelopes.size;
        yInfo() << "Number of runs    " << numberOfRun;

        envelopes.allocateStuff();
        envelopes.initializeStuff();
        while(!yarp::os::Network::isConnected(envProducer_SynchPort_name, envConsumer_SynchPort_name ))
            Time::delay(1);

        synchMsg.clear();
        synchMsg.addInt(envelopes.size);
        synchMsg.addInt(numberOfRun);
        synchPort.write(synchMsg);

        return true;
    }


    bool sendEnvelope()
    {
        Bottle reply;
        double tic, toc, taaaac;
        double timeEnv = yarp::os::Time::now();

        data.clear();
        data.addString("DATA");
        data.addInt(counter);
        data.addDouble(timeEnv);

        for(int i=0; i<envelopes.roba.size(); i++)
        {
            auto &elem = envelopes.roba[i];
            if(!quiet) std::cout << "\n---------------------------" << std::endl;
            synchMsg.clear();
            synchMsg.addString(std::get<1>(elem));
            synchPort.write(synchMsg, reply);

            // Advertize which type of envelope we are sending
            if(!quiet)  std::cout << "Sending env of type " << std::get<1>(elem) << std::endl;

            if(plain)
            {
                tic = Time::now();
                portSimple.setEnvelope(*(std::get<0>(elem)));
                toc = Time::now();
                portSimple.write(*(std::get<0>(elem)));
                taaaac = Time::now();
            }
            else
            {
                Bottle &dataBuff = portBuff.prepare();
                dataBuff.clear();
                dataBuff = data;

                tic = Time::now();
                portBuff.setEnvelope(*(std::get<0>(elem)) );
                toc = Time::now();
                portBuff.write();
                taaaac = Time::now();
            }
            if(!quiet) yInfo() << "Wrote data " << data.toString();

            std::get<3>(elem).setEnv += toc - tic;
            std::get<3>(elem).read   += taaaac - toc;
            std::get<3>(elem).tot    += taaaac - tic;

//             yInfo() << "setEnvelope elapsed time " << std::get<3>(elem).setEnv << " write " << std::get<3>(elem).read;
//             printf("Type, %15s,  setEnv, %12.9f,  read, %12.9f,  tot, %12.9f\n", std::get<1>(envelopes.roba[i]).c_str(), std::get<3>(envelopes.roba[i]).setEnv, std::get<3>(envelopes.roba[i]).read, std::get<3>(envelopes.roba[i]).tot);

//             double wait = 0.1;
//             Time::delay(wait);
        }
//         if(counter %100 == 0)
//             Time::delay(2);
    }

    bool updateModule()
    {
        if(!quiet) std::cout << "\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n" << std::endl;
        sendEnvelope();
        counter++;

        if(counter < numberOfRun)
            return true;
        else
        {
            // Print report
            Timings report;
            Bottle  rep;
            rep.clear();
            for(auto i = 0; i< envelopes.roba.size(); i++)
            {
                report = std::get<3>(envelopes.roba[i]);
                rep.addString(std::get<1>(envelopes.roba[i]));
                rep.addDouble(report.setEnv/counter);
                rep.addDouble(report.write/counter);
                rep.addDouble(report.tot/counter);

                printf("Type, %15s,  setEnv, %12.9f,  write, %12.9f,  tot, %12.9f\n", std::get<1>(envelopes.roba[i]).c_str(), report.setEnv/counter, report.write/counter, report.tot/counter);
            }

            synchPort.write(rep);
            yInfo() << rep.toString();
            return false;
        }
    }

    bool interruptModule()
    {
        yTrace();
        if(plain)
            portSimple.interrupt();
        else
            portBuff.interrupt();

        synchPort.close();
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


