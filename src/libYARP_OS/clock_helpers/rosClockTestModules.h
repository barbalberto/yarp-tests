/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef _YARP_CLOCK_TEST_H
#define _YARP_CLOCK_TEST_H

#include <string>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <rtf/yarp/YarpTestCase.h>


/**
* \ingroup yarp-tests
* Check if the clock is working as expected
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | blablabla      | string | -     | "BlaBlaBla  " | Obviously|  Nonsense   | -     |
*
*/
class ClockTest : public YarpTestCase
{
public:
    ClockTest();
    virtual ~ClockTest();

    // Test interface
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();

private:
    std::string clockPortName;
    int measure_time;
    int expected_frequency;
    int tolerance;
    yarp::os::BufferedPort<yarp::sig::Image> port;
};

#endif //_YARP_CLOCK_TEST_H
