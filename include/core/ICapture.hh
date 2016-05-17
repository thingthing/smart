#ifndef   _ICAPTURE_HH_
# define  _ICAPTURE_HH_

#include <string>

#include "event/Dispatcher.h"
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/openni2_grabber.h>
#include <opencv2/opencv.hpp>
#include "capture/real_sense_grabber.h"


class   ICapture : public Utils::Dispatcher
{
public:


  ICapture();
  virtual ~ICapture();

  pcl::PointCloud<pcl::PointXYZRGBA> const &getData() const;
  cv::Mat getDepthMat() const { return _data.depthMat; }
  cv::Mat getRgbMat() const { return _data.rgbMat; }
  virtual void captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) = 0;
  virtual void captureDataImageAndDepthOpenni(const pcl::io::openni2::Image::Ptr  &, const pcl::io::openni2::DepthImage::Ptr &, float constant) = 0;
  virtual void captureDataImageAndDepthRealSense(const boost::shared_ptr< Image > &, const boost::shared_ptr< Depth > &, float focal) = 0;
  virtual void startCapture() = 0;
  virtual void stopCapture() = 0;

  struct DATA
  {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	cv::Mat descriptions;
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat inlier;
  	cv::Mat depthMat;
  	cv::Mat rgbMat;
  };

protected:
	struct DATA 	_data; 
};


#endif    /* !_ICAPTURE_HH_ */
