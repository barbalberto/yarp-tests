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
#include <yarp/os/ConstString.h>

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
        ClockTestModule()
        {
            counter = 0;
//             yarp::os::Time::useNetworkClock("/clock");
        }
        int counter;
        bool configure(yarp::os::ResourceFinder &rf)
        {
            yInfo() << "Configure() useNetworkClock";
//             yarp::os::Time::useNetworkClock("/clock");
        }

        bool updateModule()
        {
            yInfo() << "updateModule: time is " << yarp::os::Time::now() << ", counter is " << counter;
            counter++;

            if(counter > 10)
            {
//                 yarp::os::Time::useNetworkClock("/clock");
                counter = 0;
                yInfo() << "Switching to networkClock";
            }
            return true;
        }
        
        bool interruptModule()
        {
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

/**************** Thread ***********************/
class ThreadTest : public yarp::os::RateThread
{
private:
    yarp::os::ConstString name;
    double startTime;
    public:
        ThreadTest() : RateThread(10) { }

        void setName(yarp::os::ConstString _name) { name = _name; }
        using RateThread::setRate;

        bool threadInit()
        {
            startTime = yarp::os::Time::now();
            yTrace() << "\n" << name << "Started at time " << startTime << " [System clock: " << SystemClock::nowSystem() <<  "]";
            return true;
        }
        void run() { yTrace() << "\n\t" << name << "Elapsed: " << yarp::os::Time::now() - startTime; };
        void threadRelease() { yTrace() << "\n\t" << name << yarp::os::Time::now(); };
};

//***********************************************/
//
//              MAIN STUFF
//
//***********************************************/
int main(int argc, char *argv[])
{
    yInfo("Resetting YARP_CLOCK variable to empty in order to start with system clock\n");
    Network::setEnvironment(yarp::os::ConstString("YARP_CLOCK"), yarp::os::ConstString(""));

    /**
     *
     * This test will switch between different clocks on each for loop.
     *
     */
    Network *yarp = new Network;
    if (!yarp->checkNetwork())
        return -1;

    for(int clockType=SYSTEM_CLOCK; clockType <CUSTOM_CLOCK; clockType++)
    {
        std::cout << "\n\n//********************************************//\n" << std::endl;
        yInfo() << "Testing "           << yarp::os::Time::getClockType();
        yInfo() << "Is system clock: "  << yarp::os::Time::isSystemClock();
        yInfo() << "Is network clock: " << yarp::os::Time::isNetworkClock();
        yInfo() << "Is custom clock: "  << yarp::os::Time::isCustomClock();
        yInfo() << "Is valid: "         << yarp::os::Time::isValid() << "\n";

        int someStuff = 10;
        double delayTime = 0.1;
        double start = yarp::os::Time::now();

        yInfo() << "Running " << someStuff << " trials of delay (" << delayTime << ")";
        for(int i = 0; i < someStuff; i++)
        {
            yarp::os::Time::delay(0.1);
            yInfo() << "iteration " << i << ": time is " << yarp::os::Time::now() - start << "\t - [System clock: " << SystemClock::nowSystem() <<  "]";;
        }

        // Test delay
        int someTime = 5;
        yInfo() << "Time is " << yarp::os::Time::now() << "\t - [System clock: " << SystemClock::nowSystem() <<  "]";
        yInfo() << "\n\nSleeping for " << someTime << " seconds";
        Time::delay(someTime);
        yInfo() << "Time is " << yarp::os::Time::now() << "\t - [System clock: " << SystemClock::nowSystem() <<  "]";


        // Test rateThread
        int testRate = 500;
        yInfo() << "\n\nSTARTING 1 thread with period " << testRate;
        ThreadTest test1; //, test2;
        test1.setName("Thread1");
        test1.setRate(testRate);

        // Run a thread for some real/simulated time (testing with system clock)
        test1.start();
        SystemClock::delaySystem(someTime);
        test1.stop();

        yInfo() << "\nStopping thread";

        //  Testing RF module
        //     ClockTestModule RFTest;
        //     RFTest.runModule();

        // Change clock type
        switch(clockType)
        {
            case SYSTEM_CLOCK:
            {
                yarp::os::Time::useNetworkClock("/clock");
                yInfo() << "Switching to Network Clock";
            }
            break;

//             case NETWORK_CLOCK:
//             {
//                 yarp::os::Time::useCustomClock(NULL);
//                 yInfo() << "Switching to Custom Clock (using NULL)";
//             }
//             break;

            case CUSTOM_CLOCK:
            default:
            {
                yarp::os::Time::useSystemClock();
                yInfo() << "Switching to System Clock";
            }
            break;
        }
    }

    std::cout << "\n\n//********************************************//\n" << std::endl;
    yInfo() << "Deleting yarp network oject";
    std::cout << "\n\n//********************************************//\n" << std::endl;
    delete yarp;


    std::cout << "\n\n//********************************************//\n" << std::endl;
    yInfo() << "Testing "           << yarp::os::Time::getClockType();
    yInfo() << "Is system clock: "  << yarp::os::Time::isSystemClock();
    yInfo() << "Is network clock: " << yarp::os::Time::isNetworkClock();
    yInfo() << "Is custom clock: "  << yarp::os::Time::isCustomClock();
    yInfo() << "Is valid: "         << yarp::os::Time::isValid() << "\n";



    std::cout << "\n\n//********************************************//\n" << std::endl;

    yInfo("Setting YARP_CLOCK variable to '/clock' in order to start with network clock\n");
    Network::setEnvironment(yarp::os::ConstString("YARP_CLOCK"), yarp::os::ConstString("/clock"));

    yarp = new Network;
    if (!yarp->checkNetwork())
        return -1;
    std::cout << "\n\n//********************************************//\n" << std::endl;

    if(0)
    {
        yInfo() << "Setting networkClock ";
        yarp::os::Time::useNetworkClock("/clock");
        ThreadTest test1; //, test2;

        int n=5;
        yInfo() << " 1) time is " << yarp::os::Time::now();
        std::cout << "Waiting " << n << " seconds." << std::endl;
        yarp::os::Time::delay(n);

        yInfo() << " 2) time is " << yarp::os::Time::now();
    //     std::cout << "Switching to systemClock ... " << std::endl;
    //     yarp::os::Time::useSystemClock();
    //     std::cout << "done" << std::endl;
    //     yarp::os::Time::isValid();

        std::cout << "Stopping threads ..." << std::endl;
        test1.stop();
    //     test2.stop();
        std::cout << "done" << std::endl;

    //     ClockTestModule RFTest;
    //     RFTest.runModule();
    }
    return 0;
}


