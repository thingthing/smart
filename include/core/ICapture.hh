#ifndef   _ICAPTURE_HH_
# define  _ICAPTURE_HH_

#include <string>

#include "event/Dispatcher.h"
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>

class   ICapture : public Utils::Dispatcher
{
public:


  ICapture();
  virtual ~ICapture();

  pcl::PointCloud<pcl::PointXYZRGBA> const &getData() const;
  virtual void captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) = 0;
  virtual void startCapture() = 0;
  virtual void stopCapture() = 0;

protected:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr          _cloud;

};


#endif    /* !_ICAPTURE_HH_ */
