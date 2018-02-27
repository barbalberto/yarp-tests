#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/PointCloud.hpp>
#include <pcl/io/pcd_io.h>
#include <iostream>


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

    yarp::sig::PointCloud<XYZ_RGBA_DATA> yarpCloud;

    yInfo()<<"L'header e'....."<<sizeof(cloud.header);
    //yarpCloud.fromExternalPC((char*) &cloud.points.at(0), PCL_POINT_XYZ_RGBA, 100, 1, cloud.is_dense);
    yarpCloud.fromExternalPC((char*) &cloud(0,0), PCL_POINT_XYZ_RGBA, 100, 1, cloud.is_dense);
    bool ok = true;
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

    if (yarpCloud.toPCL(cloud2.points, cloud2.width, cloud2.height))
    {
        ok = true;
        yInfo()<<"Copy successfull";
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
    }
    else
    {
        yError()<<"Copy failed";
    }

    return 0;
}
