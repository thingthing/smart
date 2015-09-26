#include "Capture.hh"

Capture::Capture()
{

}

Capture::~Capture()
{

}

pcl::PointCloud<pcl::PointXYZ> const &Capture::captureData()
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("pcdData/data/tutorials/ism_test_cat.pcd", _cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    exit(1);
  }
  std::cerr << "Loaded "
            << _cloud.width * _cloud.height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  // for (size_t i = 0; i < _cloud.points.size (); ++i)
  //   std::cerr << "    " << _cloud.points[i].x
  //             << " "    << _cloud.points[i].y
  //             << " "    << _cloud.points[i].z << std::endl;
  return (_cloud);
}