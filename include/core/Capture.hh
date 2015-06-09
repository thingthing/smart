#ifndef   _CAPTURE_HH_
# define  _CAPTURE_HH_

#include <string>

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>

class   Capture
{
public:


  Capture();
  ~Capture();

  pcl::PointCloud<pcl::PointXYZ> const &getData() const;
  pcl::PointCloud<pcl::PointXYZ> const &captureData();

private:
  pcl::PointCloud<pcl::PointXYZ>          _cloud;

};


#endif    /* !_CAPTURE_HH_ */
