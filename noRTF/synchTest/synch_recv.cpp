/**
*******************************************************************************
* This program is MY software; you canNOT redistribute it and/or
* modify it under any weather conditions!!
* This program is absolutely not useful in any possible way
* and it'll never be, so stop trying to copy it and do you homework!!
* See the 'The Hitchhiker's Guide to the Galaxy' should you fall in panic.
*******************************************************************************
*/

#include <string>
#include <cstdio>
#include <deque>
#include <stdint.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Portable.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PortSynchronizer.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


class MyType : public yarp::os::Portable
{
public:

    int         my_int;
    double      my_double;

    bool read(yarp::os::ConnectionReader& connection)
    {
        my_int    = connection.expectInt();
        my_double = connection.expectDouble();
        yDebug("read:\nmy_int %d\nmy_double %f\n", my_int, my_double);
    }

    bool write(yarp::os::ConnectionWriter& connection)
    {
        connection.appendInt(my_int);
        connection.appendDouble(my_double);
        yDebug("write:\nmy_int %d\nmy_double %f\n", my_int, my_double);
    }

    yarp::os::ConstString toString()
    {
        char str[100];
        sprintf(str, "[%d]  <--> [%f]", my_int, my_double);
        return str;
    }
};



/**************** Thread ***********************/
class SynchTestModule : public RFModule
{
private:

    int _size;
    int counter;

    yarp::os::Port                              bottle_Port_in;
    yarp::os::Port                              vector_Port_in;
    yarp::os::Port                              myType_Port_in;

//     yarp::os::Port                              image_Port_in;
//     yarp::os::Port                              flexImage_Port_in;
//     yarp::os::Port                              string_Port_in;
//     yarp::os::Port                              vectInt8_Port_in;

    yarp::dev::PortSynchronizer/*<yarp::os::Portable>*/  synchronizer;

public:
    SynchTestModule() : _size(10), counter(0)
    { }


    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        bool ok = true;
        yarp::os::ConstString suffix_i(":i");
        yarp::os::ConstString suffix_o(":o");


        ok &= bottle_Port_in .open("/bottle"+suffix_i);
        ok &= vector_Port_in .open("/vector"+suffix_i);
        ok &= myType_Port_in .open("/myType"+suffix_i);

//         ok &= vectInt8_Port_in .open("/vectInt8"+suffix_i);
//         ok &= image_Port_in    .open("/image"+suffix_i);
//         ok &= flexImage_Port_in.open("/flexImage"+suffix_i);
//         ok &= string_Port_in.open("/string"+suffix_i);


        synchronizer.addPort(bottle_Port_in);
        synchronizer.addPort(vector_Port_in);
        synchronizer.addPort(myType_Port_in);

//         synchronizer.addPort(vectInt8_Port_in);
//         synchronizer.addPort(image_Port_in);
//         synchronizer.addPort(flexImage_Port_in);
//         synchronizer.addPort(string_Port_in);

        yarp::os::Network::connect("/bottle"+suffix_o,    "/bottle"+suffix_i);
        yarp::os::Network::connect("/vector"+suffix_o,    "/vector"+suffix_i);
        yarp::os::Network::connect("/myType"+suffix_o,    "/myType"+suffix_i);

//         yarp::os::Network::connect("/vectInt8"+suffix_o,  "/vectInt8"+suffix_i);
//         yarp::os::Network::connect("/image"+suffix_o,     "/image"+suffix_i);
//         yarp::os::Network::connect("/flexImage"+suffix_o, "/flexImage"+suffix_i);
//         yarp::os::Network::connect("/string"+suffix_o, "/string"+suffix_i);
        return ok;
    }

    bool updateModule()
    {
        Bottle bb;
        Vector vv;
        MyType myT;

//         yarp::os::Time::delay(1);
//         synchronizer.getData(bb, 0);
//         yWarning() << "First data: Bottle\n\t" << bb.toString();
//         synchronizer.getData(vv, 1);
//         yWarning() << "Second data: Vector\n\t" << vv.toString();

        yarp::sig::VectorOf<yarp::os::PortReader *> v;
        v.push_back(&bb);
        v.push_back(&vv);
        v.push_back(&myT);

        synchronizer.getData(v);
        yWarning() << "First  data: Bottle\n\t" << bb.toString();
        yWarning() << "Second data: Vector\n\t" << vv.toString();
        yWarning() << "Third  data: MyType\n\t" << myT.toString();

//         yInfo() << "bottle: " << bottle_Port.read()->toString();
//         yInfo() << "vector: " << vector_Port.read()->toString();
//         yInfo() << "image : " << image_Port.read()->getPixelCode();
//         synchronizer.read();

        return true;
    }

    bool interruptModule()
    {
        yTrace();
        return true;
    }

    bool close()
    {
        yTrace();
        bottle_Port_in.close();
        vector_Port_in   .close();

//         vectInt8_Port_in .close();
//         image_Port_in    .close();
//         flexImage_Port_in.close();
//         myType_Port_in   .close();
    }
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

    Network *yarp = new Network;
    if (!yarp->checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);

    SynchTestModule module;
    yInfo() << "Configure module & start module...";
    if (!module.configure(rf))
    {
        yError() << "Error module did not start";
        return 1;
    }

    yInfo() << "Module correctly configured";

    module.runModule();
    yInfo() << "Quitting";
    return 0;
}


