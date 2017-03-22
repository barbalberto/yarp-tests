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

#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

int nJoints;
void testPositionControl(IPositionControl2   *pos);
void testPositionDirect(IPositionDirect      *posDir);
void testVelocityControl2(IVelocityControl2  *vel2);
void testAmplifierControl(IAmplifierControl  *amp);


/***********************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    bool second = false;
    std::string  prefix;
    int start = 0;
    PolyDriver remote;
    Property config;

    config.put("device", "remote_controlboard");
    config.put("remote", "/fakeMotionControl");
    config.put("local",  "/testMotionControlInterface_YARP");
//     config.put("writeStrict", "on");
    remote.open(config);

    IAmplifierControl *amp;
    IPositionControl2 *pos;
    IPositionDirect   *posDir;
    IVelocityControl  *vel;
    IVelocityControl2 *vel2;
    IPidControl       *pid;
    IControlMode2     *mode;
    IInteractionMode  *inter;

    remote.view(amp);
    remote.view(pos);
    remote.view(posDir);
    remote.view(pid);
    remote.view(mode);
    remote.view(inter);
    remote.view(vel2);

    if(!amp || !pos || !posDir || !pid || !mode || !inter || !vel2)
    {
        yError() << "Not valid interfaces found!!";
        return false;
    }

    pos->getAxes(&nJoints);
//     testPositionControl(pos);
//     testPositionDirect(posDir);
//     testVelocityControl2(vel2);
    testAmplifierControl(amp);
    return 0;
}

void testPositionControl(IPositionControl2 *pos)
{
    yarp::sig::VectorOf<int> joints;
    yarp::sig::Vector ref_sent, ref_got;

    joints.resize(nJoints);
    ref_sent.resize(nJoints);
    ref_got.resize(nJoints);

    for(int i=0; i<nJoints; i++)
    {
        joints[i] = i;
        ref_sent[i] = i* 10;
    }

    bool tot_ok = true;
    bool fail = true;

    /*
     *       POS MOVE - SINGLE JOINT
     */

    yInfo() << "Sending positions to target - single joint!!";
    for(int i=0; i<nJoints; i++)
    {
        pos->positionMove(i, ref_sent[i]);
    }

    yInfo() << "Reading back references to check consistency - single joint!!";
    tot_ok = true;
    fail = true;
    for(int i=0; i<nJoints; i++)
    {
        pos->getTargetPosition(i, &ref_got[i]);
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";


    /*
     *       POS MOVE - ALL JOINTS
     */

    for(int i=0; i<nJoints; i++)
    {
        ref_sent[i] = 10 + i;
    }

    yInfo() << "Sending positions to target - ALL joints!!";
    pos->positionMove(ref_sent.data());

    yInfo() << "Reading back references to check consistency - all joints!!";
    tot_ok = true;
    fail = true;
    pos->getTargetPositions(ref_got.data());

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";

    /*
     *       POS MOVE - GROUP JOINTS
     */
    yInfo() << "Sending positions to target - group of joints!!";
    for(int i=0; i<nJoints; i++)
    {
        joints[i] = nJoints - i -1;
        ref_sent[i] = (666 + i * 3)/14;
    }

    pos->positionMove(nJoints, joints.getFirst(), ref_sent.data());

    yInfo() << "Reading back references to check consistency - group of joints!!";
    tot_ok = true;
    fail = true;
    pos->getTargetPositions(nJoints, joints.getFirst(), ref_got.data());

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }
    yInfo() << "Values read are: " << ref_got.toString();

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";
}

/***************************************************************************************************************************************/
void testPositionDirect( IPositionDirect *posDir)
{
    yarp::sig::VectorOf<int> joints;
    yarp::sig::Vector ref_sent, ref_got;

    joints.resize(nJoints);
    ref_sent.resize(nJoints);
    ref_got.resize(nJoints);

    for(int i=0; i<nJoints; i++)
    {
        joints[i] = i;
        ref_sent[i] = i* 10;
    }

    bool tot_ok = true;
    bool fail = true;

    /*
     *       POS DIRECT - SINGLE JOINT
     */

    yInfo() << "Sending positions DIRECT to target - single joint!!";
    for(int i=0; i<nJoints; i++)
    {
        posDir->setPosition(i, ref_sent[i]);
    }
    yarp::os::Time::delay(0.2);

    yInfo() << "Reading back references to check consistency - single joint!!";
    tot_ok = true;
    fail = true;
    for(int i=0; i<nJoints; i++)
    {
        posDir->getRefPosition(i, &ref_got[i]);
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";


    /*
     *       POS MOVE - ALL JOINTS
     */

    for(int i=0; i<nJoints; i++)
    {
        ref_sent[i] = 10 + i;
    }

    yInfo() << "Sending positions DIRECT to target - ALL joints!!";
    posDir->setPositions(ref_sent.data());
    yarp::os::Time::delay(0.2);

    yInfo() << "Reading back references to check consistency - all joints!!";
    tot_ok = true;
    fail = true;
    posDir->getRefPositions(ref_got.data());

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";

    /*
     *       POS MOVE - GROUP JOINTS
     */
    yInfo() << "Sending positions DIRECT to target - group of joints!!";
    for(int i=0; i<nJoints; i++)
    {
        joints[i] = nJoints - i -1;
        ref_sent[i] = (666 + i * 3)/14;
    }

    posDir->setPositions(nJoints, joints.getFirst(), ref_sent.data());
    yarp::os::Time::delay(0.2);

    yInfo() << "Reading back references to check consistency - group of joints!!";
    tot_ok = true;
    fail = true;
    posDir->getRefPositions(nJoints, joints.getFirst(), ref_got.data());

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";
}

void testVelocityControl2(IVelocityControl2 *vel2)
{
    yarp::sig::VectorOf<int> joints;
    yarp::sig::Vector ref_sent, ref_got;

    joints.resize(nJoints);
    ref_sent.resize(nJoints);
    ref_got.resize(nJoints);

    for(int i=0; i<nJoints; i++)
    {
        joints[i] = i;
        ref_sent[i] = i* 10;
    }

    bool tot_ok = true;
    bool fail = true;

    /*
     *       VEL MOVE - SINGLE JOINT
     */

    yInfo() << "Sending velocityMove to target - single joint!!";
    for(int i=0; i<nJoints; i++)
    {
        vel2->velocityMove(i, ref_sent[i]);
    }
    yarp::os::Time::delay(0.2);

    yInfo() << "Reading back references to check consistency - single joint!!";
    tot_ok = true;
    fail = true;
    for(int i=0; i<nJoints; i++)
    {
        vel2->getRefVelocity(i, &ref_got[i]);
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";


    /*
     *       VEL MOVE - ALL JOINTS
     */

    for(int i=0; i<nJoints; i++)
    {
        ref_sent[i] = 10 + i;
    }

    yInfo() << "Sending velocityMove to target - ALL joints!!";
    vel2->velocityMove(ref_sent.data());
    yarp::os::Time::delay(0.2);

    yInfo() << "Reading back references to check consistency - all joints!!";
    tot_ok = true;
    fail = true;
    vel2->getRefVelocities(ref_got.data());

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";

    /*
     *       VEL MOVE - GROUP JOINTS
     */
    yInfo() << "Sending velocityMove to target - group of joints!!";
    for(int i=0; i<nJoints; i++)
    {
        joints[i] = nJoints - i -1;
        ref_sent[i] = (666 + i*33)/14;
    }

    vel2->velocityMove(nJoints, joints.getFirst(), ref_sent.data());
    yarp::os::Time::delay(0.2);

    yInfo() << "Reading back references to check consistency - group of joints!!";
    tot_ok = true;
    fail = true;

    vel2->getRefVelocities(nJoints, joints.getFirst(), ref_got.data());

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (ref_got[i] != ref_sent[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", joints[i],  ref_sent[i], ref_got[i]);
        }
        tot_ok &= !fail;
    }

    vel2->getRefVelocities(ref_got.data());
    yWarning() << "all: " << ref_got.toString();

    yDebug() << "refs sent are: " << ref_sent.toString();
    yDebug() << "refs got  are: " << ref_got.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";
}


void testAmplifierControl(IAmplifierControl* amp)
{
    bool tot_ok = true;
    bool fail = true;
    yarp::sig::Vector refs, reads;
    refs.resize(nJoints);
    reads.resize(nJoints);

    for(int i=0; i<nJoints; i++)
    {
        amp->getNominalCurrent(i, &reads[i]);
    }
    std::cout << "Nominal currents are " << reads.toString() << std::endl;

    /*************************************************************************/
    for(int i=0; i<nJoints; i++)
    {
        amp->getPWM(i, &reads[i]);
    }
    std::cout << "PWMs are " << reads.toString() << std::endl;

    /*************************************************************************/
    for(int i=0; i<nJoints; i++)
    {
        amp->getPowerSupplyVoltage(i, &reads[i]);
    }
    std::cout << "Power supply voltages are " << reads.toString() << std::endl;

    /*************************************************************************/
    std::cout << "Setting peak currents" << std::endl;
    for(int i=0; i<nJoints; i++)
    {
        refs[i] = 5+ (i*i*10);
        amp->setPeakCurrent(i, refs[i]);
    }

    yInfo() << "Reading back references to check consistency - group of joints!!";
    tot_ok = true;
    fail = true;
    reads.zero();
    for(int i=0; i<nJoints; i++)
    {
        amp->getPeakCurrent(i, &reads[i]);
    }

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (reads[i] != refs[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  refs[i], reads[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << refs.toString();
    yDebug() << "refs got  are: " << reads.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";


    yInfo() << "Setting PWM limits currents";
    for(int i=0; i<nJoints; i++)
    {
        refs[i] = 420+i;
        amp->setPWMLimit(i, refs[i]);
    }

    yInfo() << "Reading back references to check consistency - group of joints!!";
    tot_ok = true;
    fail = true;
    reads.zero();
    for(int i=0; i<nJoints; i++)
    {
        amp->getPWMLimit(i, &reads[i]);
    }

    for(int i=0; i<nJoints; i++)
    {
        if(fail = (reads[i] != refs[i]))
        {
            yError("Reference check failed for joint %d!! Sent %0.2f, got %0.2f", i,  refs[i], reads[i]);
        }
        tot_ok &= !fail;
    }

    yDebug() << "refs sent are: " << refs.toString();
    yDebug() << "refs got  are: " << reads.toString();

    if(tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";
}

