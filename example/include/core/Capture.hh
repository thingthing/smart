#ifndef   _CAPTURE_HH_
# define  _CAPTURE_HH_

#include <string>

#include <pcl-1.8/pcl/io/openni2_grabber.h>
#include <pcl-1.8/pcl/common/common.h>
#include <pcl-1.8/pcl/impl/point_types.hpp>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/io/openni_grabber.h>
#include <pcl-1.8/pcl/io/oni_grabber.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/common/projection_matrix.h>

#include "ICapture.hh"

class   Capture : public ICapture
{
public:

  Capture();
  virtual ~Capture();

  virtual void captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

private:
  pcl::Grabber *_grabber;
};


#endif    /* !_CAPTURE_HH_ */
