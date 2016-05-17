#ifndef   _CAPTURE_HH_
# define  _CAPTURE_HH_

#include <string>

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
// #include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "ICapture.hh"

class   Capture : public ICapture
{
public:

  Capture(std::string const &grabber_name = "OpenNI2Grabber");
  virtual ~Capture();

  virtual void captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);
  virtual void captureDataImageAndDepthOpenni(const pcl::io::openni2::Image::Ptr  &, const pcl::io::openni2::DepthImage::Ptr &, float constant);
  virtual void captureDataImageAndDepthRealSense(const boost::shared_ptr< Image > &, const boost::shared_ptr< Depth > &, float focal);
  virtual void startCapture();
  virtual void stopCapture();

private:
  pcl::Grabber *_grabber;

};


#endif    /* !_CAPTURE_HH_ */
