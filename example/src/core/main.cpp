// #include <pcl-1.7/pcl/io/openni_grabber.h>
// #include <pcl-1.7/pcl/visualization/cloud_viewer.h>

// #include <pcl-1.7/pcl/point_cloud.h>
// #include <pcl-1.7/pcl/point_types.h>
// #include <pcl-1.7/pcl/io/openni2_grabber.h>
// #include <pcl-1.7/pcl/io/pcd_io.h>
// #include <pcl-1.7/pcl/filters/statistical_outlier_removal.h>
// #include <pcl-1.7/pcl/filters/voxel_grid.h>
// #include <pcl-1.7/pcl/common/transforms.h>
// #include <vector>

// int i = 0;
// char buf[4096];

// class SimpleOpenNIViewer
// {
// public:
//     SimpleOpenNIViewer () : viewer("SMART Forum EIP") {

//     }

//     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
//     {
//         if (!viewer.wasStopped())
//         {
//             pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudClean(new pcl::PointCloud<pcl::PointXYZRGBA>());
//             // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
//             // sor.setInputCloud(cloud);
//             // sor.setMeanK(50);
//             // sor.setStddevMulThresh(1.0);
//             // sor.filter(*cloudClean);
//             // pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
//             // vox.setInputCloud(cloud);
//             // vox.setLeafSize(0.05f, 0.05f, 0.05f);
//             // vox.filter(*cloudClean);

//             // float theta = M_PI / 2; // The angle of rotation in radians
//             // Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
//             // transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
//             // pcl::transformPointCloud (*cloudClean, *cloudClean, transform_1);

//             viewer.showCloud (cloud);
// //      pcl::PCDWriter w;
//             sprintf (buf, "frame_%06d.pcd", i);
// //      w.writeBinaryCompressed (buf, *cloud);
//             // PCL_INFO ("Wrote a cloud with %zu (%ux%u) points in %s from cloud with %zu (%ux%u)points.\n", cloudClean->size (),
//             //           cloudClean->width, cloudClean->height, buf, cloud->size (),
//             //           cloud->width, cloud->height);
//             ++i;
//         }

//     }

//     void run ()
//     {
//         //"pcdData/test.oni", true, true
//         pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(); //set for streaming

//         boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

//         boost::signals2::connection c = interface->registerCallback (f);

//         interface->start ();

//         while (!viewer.wasStopped())
//         {
//             //viewer.spinOnce (100);
//             //interface->start();//to update each frame from the oni file
//             boost::this_thread::sleep (boost::posix_time::millisec (500));
//         }
//         PCL_INFO ("Successfully processed %d frames.\n", i);
//         interface->stop ();
//     }

//     pcl::visualization::CloudViewer viewer;
// };

// int main ()
// {
//     SimpleOpenNIViewer v;
//     std::cout << "HEY" << std::endl;
//     v.run ();
//     return 0;
// }



#include "Core.h"
#include <movement/Movement.h>

int     main(int argc, char **argv)
{
    Movement agentMove;

    agentMove.connectArduinoSerial();

    return 0;

    Network::NetworkManager  networkAdapter;
    AgentProtocol            protocol(networkAdapter);
    Core                     core(protocol);
    std::string              server_ip = "54.148.17.11";

    if (argc > 1)
        server_ip = argv[1];
    std::cout << "Server ip is " << server_ip << std::endl;
    /**
     * @todo : Add port in configuration files
     */
    if (networkAdapter.connectTo(server_ip, 4200, AgentProtocol::TCP_KEY, core.getAgent()) == false)
    {
        std::cout << "failed to connect tcp" << std::endl;
        return (-1);
    }
    if (networkAdapter.connectTo(server_ip, 4300, AgentProtocol::UDP_KEY, core.getAgent()) == false)
    {
        std::cout << "failed to connect udp" << std::endl;
        return (-1);
    }
    std::cout << "Starting networkAdapter" << std::endl;
    networkAdapter.start();
    std::cout << "Runing core" << std::endl;
    core.run();
    //If code is here, you are exited, so send the disconnect info to server
    networkAdapter.disconnect();
    return (0);
}
