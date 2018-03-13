

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
    Bottle bott;
    Property prop;
    porta.open("/porta2");
    prop.fromCommand(argc, argv);

    string carrier = "udp";
    if(prop.check("carrier"))
        carrier = prop.find("carrier").asString();

//    printf("Press key\n"); getchar();

//    yarp::os::Network::connect("/porta2", "/porta1", carrier);

    porta.addOutput("/porta1");
//    printf("Press key\n"); getchar();
    Time::delay(0.3);
    bott.addString("Ciao");
    porta.write(bott);

//    printf("Press key\n"); getchar();
    return 0;
}
