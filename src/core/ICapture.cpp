#include "ICapture.hh"

ICapture::ICapture()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	_data.cloud = tmp_cloud;
}

ICapture::~ICapture()
{

}

pcl::PointCloud<pcl::PointXYZRGBA> const &ICapture::getData() const
{
  return (*_data.cloud);
}
