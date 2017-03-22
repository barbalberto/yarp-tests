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

bool openMotionControl = true;
int nJoints;
void testPositionControl(IPositionControl2   *pos);
void testPositionDirect(IPositionDirect      *posDir);
void testVelocityControl2(IVelocityControl2  *vel2);
void testAmplifierControl(IAmplifierControl  *amp);
void testCurrentControl(ICurrentControl  *icurr);
void testPWMControl(IPWMControl *iPwm);

class FakeMotionControl
{
    PolyDriverList motionControllerList;
    PolyDriver motionController;
    Property controllerconfig;
    PolyDriver wrapper;
    Property  wrapperconfig;
    IMultipleWrapper* iWrap;

    public:
    FakeMotionControl()
    {
        iWrap = 0;
    }

    bool open()
    {
        const char *inifile1 = "device fakeMotionControl\n"
            "name fake\n"
            "[GENERAL]\n"
            "Joints 2\n"
            "\n"
            "AxisName \"axisA1\" \"axisA2\" \n";
        controllerconfig.fromConfig(inifile1);
        if (!motionController.open(controllerconfig)) return false;
        yarp::dev::PolyDriverDescriptor pol;
        pol.key = "fake";
        pol.poly = &motionController;
        motionControllerList.push(pol);

        yarp::os::Time::delay(1.0);

        const char *inifile2 = "device controlboardwrapper2\n"
            "period 10\n"
            "name /fakeMotionControl/fake\n"
            "joints 2\n"
            "networks ( fake )\n"
            "fake  (0 1 0 1)\n";
        wrapperconfig.fromConfig(inifile2);
        if (!wrapper.open(wrapperconfig)) return false;
        yarp::os::Time::delay(1.0);

        if (!wrapper.view(iWrap)) return false;
        if (!iWrap->attachAll(motionControllerList)) return false;

        return true;
    }
};

/***********************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;
    
    FakeMotionControl fakecontrol;
    if (openMotionControl)
    {
        fakecontrol.open();
    }

    bool second = false;
    std::string  prefix;
    int start = 0;
    PolyDriver client;
    Property clientconfig;
    
    clientconfig.put("device", "remote_controlboard");
    clientconfig.put("remote", "/fakeMotionControl/fake");
    clientconfig.put("local", "/testMotionControlInterface_YARP/fake");
//     config.put("writeStrict", "on");
    client.open(clientconfig);

    IAmplifierControl *amp = 0;
    IPositionControl2 *pos = 0;
    IPositionDirect   *posDir = 0;
    IVelocityControl  *vel = 0;
    IVelocityControl2 *vel2 = 0;
    IPidControl       *pid = 0;
    IControlMode2     *mode = 0;
    IInteractionMode  *inter = 0;
    IPWMControl       *ipwm = 0;
    ICurrentControl   *icurr = 0;

    client.view(amp);
    client.view(pos);
    client.view(posDir);
    client.view(pid);
    client.view(mode);
    client.view(inter);
    client.view(vel2);
    client.view(ipwm);
    client.view(icurr);

    if(!amp || !pos || !posDir || !pid || !mode || !inter || !vel2 || !ipwm || !icurr)
    {
        yError() << "Not valid interfaces found!!";
        return false;
    }

    pos->getAxes(&nJoints);
//     testPositionControl(pos);
//     testPositionDirect(posDir);
//     testVelocityControl2(vel2);
//     testAmplifierControl(amp);
    testPWMControl(ipwm);
    testCurrentControl(icurr);
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

void testPWMControl(IPWMControl* ipwm)
{
    bool tot_ok = true;
    bool fail = true;
    yarp::sig::Vector refs, reads;
    double* reads2=0;
    reads2 = new double[nJoints];

    refs.resize(nJoints);
    reads.resize(nJoints);

    /*************************************************************************/
    for (int i = 0; i<nJoints; i++)
    {
        ipwm->getDutyCycle(i, &reads[i]);
    }
    std::cout << "Duty cycles: " << reads.toString() << std::endl;

    /*************************************************************************/
    ipwm->getDutyCycles(reads2);
    std::cout << "Duty cycles: " << yarp::sig::Vector(nJoints,reads2).toString() << std::endl;

    /*************************************************************************/
    for (int i = 0; i<nJoints; i++)
    {
        refs[i] = i * 100 + 10;
        ipwm->setRefDutyCycle(i, refs[i]);
        yarp::os::Time::delay(0.1);
        ipwm->getRefDutyCycle(i, &reads[i]);

        if (reads[i] != refs[i]) tot_ok = false;
    }

    /*************************************************************************/
    for (int i = 0; i < nJoints; i++)
    {
        refs[i] = i * 100 + 15;
    }
    ipwm->setRefDutyCycles(refs.data());
    yarp::os::Time::delay(0.1);
    ipwm->getRefDutyCycles(reads2);
    for (int i = 0; i < nJoints; i++)
    {
        if (reads2[i] != refs[i]) tot_ok = false;
    }
    
    /*************************************************************************/
    for (int i = 0; i<nJoints; i++)
    {
        ipwm->getDutyCycle(i, &reads[i]);
    }
    std::cout << "Duty cycles: " << reads.toString() << std::endl;

    /*************************************************************************/
    ipwm->getDutyCycles(reads2);
    std::cout << "Duty cycles: " << yarp::sig::Vector(nJoints, reads2).toString() << std::endl;

    /*************************************************************************/
    if (tot_ok)
        yInfo() << "TEST OK";
    else
        yError() << "TEST FAILED";
}

void testCurrentControl(ICurrentControl* icurr)
{
  
}

