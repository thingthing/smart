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

int main ()
{
    SimpleOpenNIViewer v;
    std::cout << "HEY" << std::endl;
    v.run ();
    return 0;
}