#include "ICapture.hh"

ICapture::ICapture()
  : _cloud(new pcl::PointCloud<pcl::PointXYZRGBA>)
{

}

ICapture::~ICapture()
{

}

pcl::PointCloud<pcl::PointXYZRGBA> const &ICapture::getData() const
{
  return (*_cloud);
}
