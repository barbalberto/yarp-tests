/**
*******************************************************************************
* This program is MY software; you canNOT redistribute it and/or
* modify it under any weather conditions!!
* This program is absolutely not useful in any possible way
* and it'll never be so stop trying to copy it and do you homework!!
* See the 'The Hitchhiker's Guide to the Galaxy' should you fall into panic.
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
};



/**************** Thread ***********************/
class SynchTestModule : public RFModule
{
private:

    int _size;
    int counter;

    yarp::os::BufferedPort<Bottle>              bottle_Port_out;
    yarp::os::BufferedPort<Vector>              vector_Port_out;
    yarp::os::BufferedPort<MyType>              myType_Port_out;

//     yarp::os::BufferedPort<ImageOf< PixelRgb> > image_Port_out;
//     yarp::os::BufferedPort<FlexImage>           flexImage_Port_out;
//     yarp::os::BufferedPort<ConstString>         string_Port_out;
//     yarp::os::BufferedPort<VectorOf<int> >      vectInt8_Port_out;

public:
    SynchTestModule() : _size(10), counter(0)
    { }


    bool configure(yarp::os::ResourceFinder &rf)
    {
        yTrace();
        bool ok = true;
        yarp::os::ConstString suffix_i(":i");
        yarp::os::ConstString suffix_o(":o");

        ok &= bottle_Port_out   .open("/bottle"+suffix_o);
        ok &= vector_Port_out   .open("/vector"+suffix_o);
        ok &= myType_Port_out   .open("/myType"+suffix_o);

//         ok &= image_Port_out    .open("/image"+suffix_o);
//         ok &= flexImage_Port_out.open("/flexImage"+suffix_o);
//         ok &= string_Port_out.open("/string"+suffix_o);
//         ok &= vectInt8_Port_out .open("/vectInt8"+suffix_o);

        return ok;
    }

    bool updateModule()
    {
        yTrace();

        Bottle &b = bottle_Port_out.prepare();
        Vector &v = vector_Port_out.prepare();
        MyType &myT = myType_Port_out.prepare();

//         VectorOf<int> &v8 = vectInt8_Port.prepare();
//         ImageOf< PixelRgb > &img = image_Port.prepare();
//         FlexImage &flex = flexImage_Port.prepare();
//         Vector &_str = string_Port.prepare();
//
        v.resize(_size);

//         v8.resize(_size);
//         img.resize(_size, _size);
//
//         flex.setPixelCode(VOCAB_PIXEL_MONO_FLOAT);
//         flex.resize(_size, _size);

        b.clear();
        b.addInt(counter);

        for(int i=0; i< _size; i++)
        {
            v[i] = counter;
//             v8[i] = counter % 255;
        }

        myT.my_int = counter;
        myT.my_double = counter;

//         memset(img.getRawImage(),  counter % 255, img.getRawImageSize());
//         memset(flex.getRawImage(), counter % 255, flex.getRawImageSize());

        yarp::os::Stamp stamp;
        stamp.update();
        bottle_Port_out.setEnvelope(stamp);
        vector_Port_out.setEnvelope(stamp);
        myType_Port_out.setEnvelope(stamp);

//         vectInt8_Port.setEnvelope(stamp);
//         image_Port.setEnvelope(stamp);
//         flexImage_Port.setEnvelope(stamp);

        bottle_Port_out   .write();
        vector_Port_out   .write();
        myType_Port_out   .write();

//         vectInt8_Port .write();
//         image_Port    .write();
//         flexImage_Port.write();

        counter++;
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
        bottle_Port_out.close();
        vector_Port_out.close();
        myType_Port_out.close();

//         vectInt8_Port .close();
//         image_Port    .close();
//         flexImage_Port.close();
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


