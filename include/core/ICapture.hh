#ifndef   _ICAPTURE_HH_
# define  _ICAPTURE_HH_

#include <string>

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>

class   ICapture
{
public:


  ICapture();
  virtual ~ICapture();

  pcl::PointCloud<pcl::PointXYZ> const &getData() const;
  virtual pcl::PointCloud<pcl::PointXYZ> const &captureData() = 0;

protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr          _cloud;

};


#endif    /* !_ICAPTURE_HH_ */
