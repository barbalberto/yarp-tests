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
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/SizedWriter.h>

#include <yarp/dev/all.h>
#include <yarp/sig/all.h>


#include "portablesBestOf.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/**************** Thread ***********************/
class TimeStamp_Consumer : public RFModule
{
private:
    bool    success;
    bool    plain;
    double  sleep;
    Bottle  synchMsg;
    ConstString branch;
    ConstString carrier;

    int     size;
    int     numOfRun;
    bool    quiet;
    Port    portPlain;
    Port    synchPort;
    yarp::os::BufferedPort<yarp::os::Bottle> portBuff;

    PortablesBestOf  data_read;
    PortablesBestOf  envelopes_read;
    PortablesBestOf  envelopes_reference;

    int counter;
public:

    TimeStamp_Consumer(bool system=true) : plain(true), sleep(0), quiet(true)
    {
        yTrace();
        counter = 0;
        numOfRun = 10;
    }

    double getPeriod()
    {
        return 0.001;
    }

    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();

        synchPort.open(envConsumer_SynchPort_name);

        carrier = "udp";
        if(rf.check("carrier"))
            carrier = rf.find("carrier").asString();

        if(rf.check("buff"))
            plain = false;

        if(rf.check("verbose"))
        {
            envelopes_read.verbose = true;
            envelopes_reference.verbose = true;
            quiet = false;
        }

        sleep = rf.check("sleep", Value(0), "sleep or not").asDouble();

        if(rf.check("branch"))
            branch = rf.find("branch").asString();
        else
        {
            yError() << " missing branch param";
            return false;
        }

        if(plain)
        {
            portPlain.open(envConsumer_DataPort_name);
            yInfo() << "<<< Using PLAIN port >>>";
        }
        else
        {
            portBuff.open(envConsumer_DataPort_name);
            yInfo() << "<<< Using BUFFERED port >>>";
        }

        while(!yarp::os::Network::connect(envProducer_SynchPort_name, envConsumer_SynchPort_name, "tcp" ))
            yarp::os::Time::delay(0.05);

        while(!yarp::os::Network::connect(envProducer_DataPort_name, envConsumer_DataPort_name, carrier) )
            yarp::os::Time::delay(0.05);

        yInfo() << "Connections done!!";

        synchPort.read(synchMsg);
        size     = synchMsg.get(0).asInt();
        numOfRun = synchMsg.get(1).asInt();

        data_read.allocateStuff();
        data_read.zeroStuff();
        envelopes_read.allocateStuff();
        envelopes_read.zeroStuff();

        // Allocate reference with same size as producer
        envelopes_reference.size = size;
        envelopes_reference.allocateStuff();
        envelopes_reference.initializeStuff();
        std::cout << "Done initializing stuff";

        yInfo() << "Number of runs    " << numOfRun;
        return true;
    }

    bool handleEnvelope()
    {
        Bottle dato;
        Bottle envelope;
        double tic, toc, taaaac;

        Bottle reply;
        reply.addString("Hello");
        Portable *elem = nullptr;
        Portable *datum = nullptr;

        for(auto i=0; i<envelopes_reference.roba.size(); i++)
        {
//             std::cout << "\n---------------------------" << std::endl;
//             std::cout << "Waiting synch message "  << std::flush;
            synchPort.read(synchMsg, true);      // don't care about reply
            synchPort.reply(reply);
//             std::cout << "\t Reply sent."  << std::endl;

            elem  = std::get<0>(envelopes_read.roba[i]);
            datum = std::get<0>(data_read.roba[i]);

//             std::cout << "Reading env of type " << std::get<1>(envelopes_read.roba[i]) << std::endl;

            if(!quiet) std::cout << "-> Receiving " << std::get<1>(envelopes_read.roba[i]) << std::endl;
            if(plain)
            {
//                 bRead = &dato;
                tic = Time::now();
                    portPlain.read(*datum);
                toc = Time::now();
                    portPlain.getEnvelope(*elem);
                taaaac = Time::now();
//                 yInfo() << "message queue size" << portPlain.getInputCount();
            }
            else    // buffered port
            {
                tic = Time::now();
                    datum = portBuff.read();
                toc = Time::now();
                    success = portBuff.getEnvelope(*elem);
                taaaac = Time::now();
//                 yInfo() << "message queue size" << portBuff.getInputCount();
            }
//             if(datum)
//                 yInfo("---Success:  %d", success);

            //
            //  Verify received data and envelope are correct
            //
/*
            yWarning("Checking DATA");
            if(std::get<2>(envelopes_reference.roba[i])(envelopes_reference, std::get<0>(envelopes_reference.roba[i]), datum))
           {
//                 data_succeded.push_back(std::get<1>(envelopes_reference.roba[i]) );
                printf("Data %s MATCH!\n\n", std::get<1>(envelopes_reference.roba[i]).c_str());
            }
            else
            {
                data_failed.push_back(std::get<1>(envelopes_reference.roba[i]) );
                printf("Data %s DO NOT match!\n\n", std::get<1>(envelopes_reference.roba[i]).c_str());
            }
*/
//             yWarning("Checking ENVELOPE");
            if(std::get<2>(envelopes_reference.roba[i])(envelopes_reference, std::get<0>(envelopes_reference.roba[i]), std::get<0>(envelopes_read.roba[i])))
            {
                std::get<4>(envelopes_read.roba[i]) = true;
                if(!quiet) printf("Envelope MATCH!\n\n");
            }
            else
            {
                if(!quiet) printf("Envelope does NOT match\n\n");
            }

            std::get<3>(envelopes_read.roba[i]).setEnv = toc - tic;
            std::get<3>(envelopes_read.roba[i]).read   = taaaac - toc;
            std::get<3>(envelopes_read.roba[i]).tot    = taaaac - tic;

//             yInfo() << "\nTiming: for read " << toc-tic << " for getEnvelope " << taaaac -toc;
        }
    }

    bool updateModule()
    {
        std::cout << "\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\nRun number " << counter+1 << std::endl;
        handleEnvelope();
        counter++;

        if(counter < numOfRun)
            return true;
        else
        {
            // Print report
/*
            yInfo() << " ** ";
            yInfo() << " ** DATA REPORT";
            yInfo() << " ** ";
            yInfo() << "Test succesful DATA " << data_succeded.size() << " over " << envelopes_reference.roba.size();
            if(data_succeded.size() != envelopes_reference.roba.size())
                yError() << "Not all tests were succesful";

            std::cout << "Succeded data are" << std::endl;
            for(auto elem : data_succeded)
            {
                std::cout << "\t" << elem << std::endl;
            }

            std::cout << "Failed data are" << std::endl;
            for(auto elem : data_failed)
            {
                std::cout << "\t" << elem << std::endl;
            }
*/
            string filename = string("Report")  + "-branch_"  + branch
                                                + "-carrier_" + carrier
                                                + (plain ? "_rpc_" : "_buff_")
                                                + "-size_" + std::to_string(size)
                                                + "-runs_" + std::to_string(numOfRun) + ".txt";

            FILE* file_p = std::fopen(filename.c_str(), "w");
            if(!file_p)
                file_p = stdout;

            yInfo() << " ** ";
            yInfo() << " ** ENVELOPE REPORT";
            yInfo() << " ** ";

            fprintf(file_p, "\n--- Succesful ENVELOPE\n");
            Timings report;
            int a = 0, b = 0;
            for(auto elem : envelopes_read.roba)
            {
                if(get<4>(elem))
                {
                    a++;
                    fprintf(file_p, "%s\n", get<1>(elem).c_str() );
                }
                b++;
            }

            fprintf(file_p, "\n--- Failed envelopes are\n");
            for(auto elem : envelopes_read.roba)
            {
                if(!get<4>(elem))
                    fprintf(file_p, "%s\n", get<1>(elem).c_str() );
            }

            fprintf(file_p, "\n Summary: %d over %d were succesful\n", a, b);

            fprintf(file_p, "\nTiming report\n");
            for(auto elem : envelopes_read.roba)
            {
                if(get<4>(elem))
                {
                    report = std::get<3>(elem);
                    fprintf(file_p, "Type, %15s,  setEnv, %12.9f,  read, %12.9f,  tot, %12.9f\n", get<1>(elem).c_str(), report.setEnv/counter, report.read/counter, report.tot/counter);
//                     printf("Type, %15s,  setEnv, %12.9f,  read, %12.9f,  tot, %12.9f\n", get<1>(elem).c_str(), report.setEnv/counter, report.read/counter, report.tot/counter);
                }
            }

            // Acquire report from the sender
            synchMsg.clear();
            synchPort.read(synchMsg);

            fprintf(file_p, "\nSummary: from Producer\n\n");

//             yInfo() << "Sender report: \n " << synchMsg.toString();

            for(int i=0; i<synchMsg.size(); i+=4)
            {
//                 printf("%s\n %12.9f \n  %12.9f\n  %12.9f\n",    synchMsg.get(i).asString().c_str(),
//                                                                 synchMsg.get(i+1).asDouble(),
//                                                                 synchMsg.get(i+2).asDouble(),
//                                                                 synchMsg.get(i+3).asDouble());
                fprintf(file_p, "Type, %15s, setEnv, %12.9f,  write, %12.9f,  tot, %12.9f\n", synchMsg.get(i).asString().c_str(), synchMsg.get(i+1).asDouble(), synchMsg.get(i+2).asDouble(), synchMsg.get(i+3).asDouble());
            }

            fclose(file_p);
            std::cout << "\n\nReport saved to file " << filename;
        }
        return false;
    }

    bool interruptModule()
    {
        yTrace();
        portPlain.interrupt();
        portBuff.interrupt();

        portPlain.close();
        portBuff.close();

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
    TimeStamp_Consumer RFTest;
    RFTest.runModule(rf);
    return 0;
}
