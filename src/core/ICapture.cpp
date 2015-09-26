#include "ICapture.hh"

ICapture::ICapture()
{

}

ICapture::~ICapture()
{

}

pcl::PointCloud<pcl::PointXYZ> const &ICapture::getData() const
{
  return (_cloud);
}
