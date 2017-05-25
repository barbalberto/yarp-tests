// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan and Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#include <iostream>
#include <stdlib.h>     // for abs()
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include "ClockTest.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;


// prepare the plugin
PREPARE_PLUGIN(ClockTest)

ClockTest::ClockTest() : YarpTestCase("ClockTest"),
                         clockPortName("/ciao") {  }

ClockTest::~ClockTest() { }

bool ClockTest::setup(yarp::os::Property& property)
{
    // Set Test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // Do stuff ...

    // updating parameters
    if( property.check("clockPortName") )
    {
        clockPortName = property.find("clockPortName").asString();
    }

    // opening port
    RTF_ASSERT_ERROR_IF(port.open("/ClockTest:i"), "opening port, is YARP network available?");

    RTF_TEST_REPORT(Asserter::format("Listening to camera for %d seconds", measure_time));

    // connecting
    RTF_TEST_REPORT(Asserter::format("connecting from %s to %s", port.getName().c_str(), clockPortName.c_str()));
    RTF_ASSERT_ERROR_IF(Network::connect(clockPortName, port.getName()), "could not connect to remote port, camera unavailable");
    return true;
}

void ClockTest::tearDown()
{
    Network::disconnect(clockPortName, port.getName());
    port.close();
}

void ClockTest::run()
{
    RTF_TEST_REPORT("Running stuff ...");

    RTF_TEST_REPORT(Asserter::format("Reporting stuff"));
    RTF_TEST_CHECK(1, "Something is succeding happily ...");
    RTF_TEST_FAIL_IF(0, "Something is failing happily ...");
}
