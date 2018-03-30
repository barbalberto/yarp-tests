#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <yarp/pcl/Pcl.h>

using namespace yarp::sig;


int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.resize(100);
    for(int i=0; i<cloud.size(); i++)
    {
        cloud.points.at(i).x = i;
        cloud.points.at(i).y = i+1;
        cloud.points.at(i).z = i+2;
        cloud.points.at(i).r = 'r';
        cloud.points.at(i).g = 'g';
        cloud.points.at(i).b = 'b';
    }

    yarp::sig::PointCloud<XYZRGBAData> yarpCloud;

    bool ok = yarp::pcl::fromPCL<pcl::PointXYZRGBA, XYZRGBAData>(cloud, yarpCloud);
    for (int i=0;i<cloud.size();i++)
    {
        ok &= yarpCloud(i).x == i;
        ok &= yarpCloud(i).y == i + 1;
        ok &= yarpCloud(i).z == i + 2;
        ok &= yarpCloud(i).r == 'r';
        ok &= yarpCloud(i).g == 'g';
        ok &= yarpCloud(i).b == 'b';

    }
    if (ok)
    {
        std::cout<<"FromPcl:Everything works"<<std::endl;
    }
    else
    {
        std::cout<<"FromPcl:Nothing works"<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZRGBA> cloud2;

    ok = yarp::pcl::toPCL<XYZRGBAData, pcl::PointXYZRGBA>(yarpCloud, cloud2);

    for (int i=0; i<cloud2.size(); i++)
    {
        ok &= cloud2.points.at(i).x == i;
        ok &= cloud2.points.at(i).y == i + 1;
        ok &= cloud2.points.at(i).z == i + 2;
        ok &= cloud2.points.at(i).r == 'r';
        ok &= cloud2.points.at(i).g == 'g';
        ok &= cloud2.points.at(i).b == 'b';
    }
    if (ok)
    {
        std::cout<<"ToPcl:Everything works"<<std::endl;
    }
    else
    {
        std::cout<<"ToPcl:Nothing works"<<std::endl;
    }

    return 0;
}
