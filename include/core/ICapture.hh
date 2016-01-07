#ifndef   _ICAPTURE_HH_
# define  _ICAPTURE_HH_

#include <string>

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
// #include <pcl-1.7/pcl/common/projection_matrix.h>
#include "event/Dispatcher.h"

class   ICapture : public Utils::Dispatcher
{
public:


  ICapture();
  virtual ~ICapture();

  pcl::PointCloud<pcl::PointXYZRGBA> const &getData() const;
  virtual void captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) = 0;

protected:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr          _cloud;

};


#endif    /* !_ICAPTURE_HH_ */
