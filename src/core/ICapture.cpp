#include "ICapture.hh"

ICapture::ICapture()
  : _cloud(new pcl::PointCloud<pcl::PointXYZ>)
{

}

ICapture::~ICapture()
{

}

pcl::PointCloud<pcl::PointXYZ> const &ICapture::getData() const
{
  return (*_cloud);
}
