#include <iostream>
#include <yarp/os/Port.h>
#include <yarp/sig/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yarp/pcl/Pcl.h>

using namespace std;
using namespace yarp::os;


int
main (int argc, char** argv)
{
    // test save
    yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> test;
    int width  = 32;
    int height = 25;
    test.resize(width, height);

    for (int i=0; i<width*height; i++)
    {
        test(i).x = i;
        test(i).y = i + 1;
        test(i).z = i + 2;
        test(i).r = '1';
        test(i).g = '2';
        test(i).b = '3';
        test(i).a = '4';
    }

    const string filename("yarp_test_pcd.pcd");
    int result = yarp::pcl::savePCD< yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA >(filename, test);

    if (result != 0)
    {
        cerr << "Save point cloud to PCD file operation failed." << endl;
        return -1;
    }
    else
    {
        cout << "Save point cloud to PCD works." << endl;
    }

    yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> testFromFile;
    result = yarp::pcl::loadPCD< pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA >(filename, testFromFile);
    if (result != 0)
    {
        cerr << "Unable to load the point cloud from the PCD file" << endl;
        return -1;
    }
    bool equality = test.dataSizeBytes() == testFromFile.dataSizeBytes(); // check size
    equality = equality && (test.height() == testFromFile.height()); // check height
    equality = equality && (test.width() == testFromFile.width()); // check width

    for (int i = 0; i < test.size(); i++)
    {
        equality = equality && (test(i).x == testFromFile(i).x &&
                                test(i).y == testFromFile(i).y &&
                                test(i).z == testFromFile(i).z &&
                                test(i).r == testFromFile(i).r &&
                                test(i).g == testFromFile(i).g &&
                                test(i).b == testFromFile(i).b &&
                                test(i).a == testFromFile(i).a); // check pixel equality
    }

    if (!equality)
    {
        cerr << "Loaded point cloud is different from saved one." << endl;
        return -1;
    }
    else
    {
        cout << "Loading point cloud from PCD works." << endl;
    }

    return 0;
}

