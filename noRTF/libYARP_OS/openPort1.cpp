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

using namespace std;
using namespace yarp::os;


/***********************************************/
int main(int argc, char *argv[])
{
    // initialize yarp with system clock
    yarp::os::Network yarp(YARP_CLOCK_SYSTEM);
    if(!yarp::os::Network::checkNetwork())
        return -1;

    Port porta;
    Bottle b;
    porta.open("/porta1");

    porta.read(b);
//    printf("Press key\n"); getchar();

    yInfo() << "Got message";

    return 0;
}
