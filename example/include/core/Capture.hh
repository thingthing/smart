#ifndef   _CAPTURE_HH_
# define  _CAPTURE_HH_

#include <string>

#include <pcl/io/openni2_grabber.h>
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
//#include "capture/real_sense_grabber.h"

#include "ICapture.hh"

class   Capture : public ICapture
{
public:

  Capture();
  virtual ~Capture();

  virtual void captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);
  virtual void startCapture();
  virtual void stopCapture();

private:
  pcl::Grabber *_grabber;
};


#endif    /* !_CAPTURE_HH_ */
