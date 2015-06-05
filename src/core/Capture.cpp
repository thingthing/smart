#include "Capture.hh"

Capture::Capture()
{

}

Capture::~Capture()
{

}

pcl::PointCloud<pcl::PointXYZ> const &Capture::getData() const
{
  return (_cloud);
}

pcl::PointCloud<pcl::PointXYZ> const &Capture::captureData()
{
  int numberSample = 500;
  _cloud.width    = numberSample;
  _cloud.height   = 1;
  _cloud.is_dense = false;
  _cloud.points.resize (_cloud.width * _cloud.height);
  for (unsigned int i = 0; i < _cloud.points.size(); ++i)
  {
    _cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    _cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    if (i % 2 == 0)
      _cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
    else
      _cloud.points[i].z = -1 * (_cloud.points[i].x + _cloud.points[i].y);
  }
  return (_cloud);
}