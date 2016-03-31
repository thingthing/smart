#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <pcl/filters/passthrough.h>
#include "capture/real_sense_grabber.h"

int i = 0;
char buf[4096];

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer("SMART Forum EIP") {

    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudClean(new pcl::PointCloud<pcl::PointXYZRGBA>());

            pcl::PassThrough<pcl::PointXYZRGBA> pass;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (1.0, 3.0);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloudClean);

            // pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
            // vox.setInputCloud(cloudClean);
            // vox.setLeafSize(0.05f, 0.05f, 0.05f);
            // vox.filter(*cloudClean);

            // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
            // sor.setInputCloud(cloudClean);
            // sor.setMeanK(50);
            // sor.setStddevMulThresh(1.0);
            // sor.filter(*cloudClean);
            std::cerr << "Filtering cloud size = " << cloudClean->size() << " -- cloud size orig" << cloud->size() << std::endl;

            float theta = M_PI; // The angle of rotation in radians
            Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
            transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
            pcl::transformPointCloud (*cloudClean, *cloudClean, transform_1);

            viewer.showCloud (cloudClean);
//      pcl::PCDWriter w;
            sprintf (buf, "frame_%06d.pcd", i);
//      w.writeBinaryCompressed (buf, *cloud);
            // PCL_INFO ("Wrote a cloud with %zu (%ux%u) points in %s from cloud with %zu (%ux%u)points.\n", cloudClean->size (),
            //           cloudClean->width, cloudClean->height, buf, cloud->size (),
            //           cloud->width, cloud->height);
            ++i;
        }

    }

    void run ()
    {
        //"pcdData/test.oni", true, true
        pcl::Grabber* interface = new RealSenseGrabber(); //set for streaming

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        boost::signals2::connection c = interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            //viewer.spinOnce (100);
            //interface->start();//to update each frame from the oni file
            boost::this_thread::sleep (boost::posix_time::millisec (500));
        }
        PCL_INFO ("Successfully processed %d frames.\n", i);
        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <map>

#include "myahrs_plus.hpp"
using namespace WithRobot;

static const int BAUDRATE = 115200;

static const char* DIVIDER = "1";  // 100 Hz


void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}

int main ()
{
    // SimpleOpenNIViewer v;
    // std::cout << "HEY" << std::endl;
    // v.run ();

    MyAhrsPlus sensor;
    SensorData sensor_data;
    uint32_t sample_count = 0;

    /*
     * 	start communication with the myAHRS+.
     */
    if(sensor.start("/dev/ttyACM0", BAUDRATE) == false) {
        handle_error("start() returns false");
    }

    /*
     *  set ascii output format
     *   - select euler angle
     */
    if(sensor.cmd_ascii_data_format("RPY") == false) {
        handle_error("cmd_ascii_data_format() returns false");
    }

    /*
     *  set divider
     *   - output rate(Hz) = max_rate/divider
     */
    if(sensor.cmd_divider(DIVIDER) ==false) {
        handle_error("cmd_divider() returns false");
    }

    /*
     *  set transfer mode
     *   - AC : ASCII Message & Continuous mode
     */
    if(sensor.cmd_mode("AC") ==false) {
        handle_error("cmd_mode() returns false");
    }

    while(sample_count < 300) {
        if(sensor.wait_data() == true) { // waiting for new data
        	// read counter
            sample_count = sensor.get_sample_count();

            // copy sensor data
            sensor.get_data(sensor_data);

            // print euler angle
            EulerAngle& e = sensor_data.euler_angle;
            printf("%04d) EulerAngle (roll = %.2f, pitch = %.2f, yaw = %.2f)\n", sample_count, e.roll, e.pitch, e.yaw);
        }
    }

    /*
     * 	stop communication
     */
    sensor.stop();

    printf("END OF TEST(%s)\n\n", __FUNCTION__);
    return 0;
}