#include "Capture.hh"
// Constant is 1/ focal length
//  void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)

Capture::Capture(std::string const &grabber_name)
{
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> getData = boost::bind (&Capture::captureData, this, _1);
  if (grabber_name == "RealSenseGrabber")
  {
    _grabber = new RealSenseGrabber();
    boost::function<RealSenseGrabber::sig_cb_real_sense_image_depth_image> getImageAndDepth = boost::bind (&Capture::captureDataImageAndDepthRealSense, this, _1, _2, _3);
    _grabber->registerCallback(getImageAndDepth);
  }
  else if (grabber_name == "OpenNI2Grabber")
  {
    _grabber = new pcl::io::OpenNI2Grabber("openni", pcl::io::OpenNI2Grabber::OpenNI_QVGA_60Hz,  pcl::io::OpenNI2Grabber::OpenNI_QVGA_60Hz);
    boost::function<pcl::io::OpenNI2Grabber::sig_cb_openni_image_depth_image> getImageAndDepth = boost::bind (&Capture::captureDataImageAndDepthOpenni, this, _1, _2, _3);
    _grabber->registerCallback(getImageAndDepth);
  }
  else
  {
    std::cerr << "Error: Grabber " << grabber_name << " is not managed" << std::endl;
    throw new std::exception();
  }
  _grabber->registerCallback(getData);
}

Capture::~Capture()
{
  _grabber->stop();
  std::cerr << "grabber delete" << std::endl;
  delete _grabber;
}

void Capture::startCapture() {
  if (!_grabber->isRunning()) {
    std::cerr << "grabber start" << std::endl;
    _grabber->start();
  }
}

void Capture::stopCapture() {
  _grabber->stop();
  std::cerr << "grabber stop" << std::endl;
}

void  Capture::captureDataImageAndDepthOpenni(const pcl::io::openni2::Image::Ptr &image, const pcl::io::openni2::DepthImage::Ptr &depth, float focal)
{
  cv::Mat tempImage = cv::Mat(image->getHeight(), image->getWidth(), CV_8UC3,
     const_cast<void *>(image->getData()));
  cv::cvtColor(tempImage, _data.rgbMat, CV_RGB2BGR);
  cv::Mat tempDepth = cv::Mat(depth->getHeight(), depth->getWidth(), CV_16UC1,
     const_cast<unsigned short *>(depth->getData()));
  tempDepth.copyTo(_data.depthMat);
//  std::cerr << "For openni Image data == " << _data.rgbMat << " -- depth == " << _data.depthMat << " -- 1/focal length == " << focal << std::endl;
}

void Capture::captureDataImageAndDepthRealSense(const boost::shared_ptr< Image > &image,
  const boost::shared_ptr< Depth > &depth, float focal)
{
  cv::Mat tempImage = cv::Mat(image->_height, image->_width, CV_8UC3, const_cast<uint8_t *>(image->_rgb));
  cv::cvtColor(tempImage, _data.rgbMat, CV_RGB2BGR);
  cv::Mat tempDepth = cv::Mat(depth->_height, depth->_width, CV_16UC1, const_cast<uint16_t *>(depth->_depth));
  tempDepth.copyTo(_data.depthMat);
  // std::cerr << "                  For realsense Image data == " << _data.rgbMat << " -- depth == " << _data.depthMat << " -- 1/focal length == " << focal << std::endl;
}


void Capture::captureData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("pcdData/data/tutorials/ism_test_cat.pcd", _cloud) == -1) //* load the file
  // {
  //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //   exit(1);
  // }
  // std::cerr << "Loaded "
  //           << _cloud.width * _cloud.height
  //           << " data points from test_pcd.pcd with the following fields: "
  //           << std::endl;
  // std::cout << "PointCloud is == " << std::endl;
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  //   std::cout << "    " << cloud->points[i].x
  //             << " "    << cloud->points[i].y
  //             << " "    << cloud->points[i].z << std::endl;
  // std::cout << "End point cloud" << std::endl;
  std::cerr << "Start capture " << cloud->size() << std::endl;
  if (!_data.cloud->empty())
      _data.cloud->clear();
  //pcl::copyPointCloud(*cloud, *_cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *_data.cloud, indices);

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  // sor.setInputCloud(_data.cloud);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*_data.cloud);
  //std::cerr << "Before  " << _cloud->size() << std::endl;
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (_data.cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.0, 3.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*_data.cloud);

  //std::cerr << "After PassThrough " << _cloud->size() << std::endl;

  // pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
  // vox.setInputCloud(_cloud);
  // vox.setLeafSize(0.05f, 0.05f, 0.05f);
  // vox.filter(*_cloud);
  std::cerr << "After VoxelGrid " << _data.cloud->size() << std::endl;

  this->dispatch("takeDataEvent");
  //boost::this_thread::sleep(boost::posix_time::millisec(10));

  // std::cerr << "Loaded "
  //           << _cloud.points.size()
  //           << " data points"
  //           << std::endl;
}
