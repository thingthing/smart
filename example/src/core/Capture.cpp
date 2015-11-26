#include "Capture.hh"

Capture::Capture()
 : _grabber(new pcl::ONIGrabber("pcdData/test.oni", false, false))
{
  boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> getData = [this](const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
  {
    pcl::copyPointCloud(*cloud, *_cloud);
  };
  _grabber->registerCallback(getData);
}

Capture::~Capture()
{
  delete _grabber;
}

pcl::PointCloud<pcl::PointXYZ> const &Capture::captureData()
{
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("pcdData/data/tutorials/ism_test_cat.pcd", _cloud) == -1) //* load the file
  // {
  //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //   exit(1);
  // }
  // std::cerr << "Loaded "
  //           << _cloud.width * _cloud.height
  //           << " data points from test_pcd.pcd with the following fields: "
  //           << std::endl;
  // for (size_t i = 0; i < _cloud.points.size (); ++i)
  //   std::cerr << "    " << _cloud.points[i].x
  //             << " "    << _cloud.points[i].y
  //             << " "    << _cloud.points[i].z << std::endl;
  _grabber->start();
  boost::this_thread::sleep(boost::posix_time::seconds(1));
  // Stop Retrieve Data
  _grabber->stop();
  // std::cerr << "Loaded "
  //           << _cloud.points.size()
  //           << " data points"
  //           << std::endl;
  return (*_cloud);
}