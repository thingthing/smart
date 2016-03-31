/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <boost/lexical_cast.hpp>

#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include <pcl/io/io_exception.h>
#include <capture/real_sense_grabber.h>
#include <capture/real_sense/real_sense_device_manager.h>


/* Helper function to convert a rs::float3 point into a PCL point.
 * Takes care of unit conversion (PXC point coordinates are in millimeters)
 * and invalid points. */
template <typename T> inline void
convertPoint (const rs::float3& src, T& tgt)
{
  //static const float nan = std::numeric_limits<float>::quiet_NaN ();
  // if (src.z == 0)
  // {
  //   tgt.x = tgt.y = tgt.z = nan;
  // }
  // else
  // {
    tgt.x = src.x;
    tgt.y = src.y;
    tgt.z = src.z;
  //}
}

// pcl::RealSenseGrabber::Mode::Mode ()
// : fps (0), depth_width (0), depth_height (0), color_width (0), color_height (0)
// {
// }

// pcl::RealSenseGrabber::Mode::Mode (unsigned int f)
// : fps (f), depth_width (0), depth_height (0), color_width (0), color_height (0)
// {
// }

// pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh)
// : fps (0), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
// {
// }

// pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh)
// : fps (f), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
// {
// }

// pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
// : fps (0), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
// {
// }

// pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
// : fps (f), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
// {
// }

// bool
// operator== (const pcl::RealSenseGrabber::Mode& m1, const pcl::RealSenseGrabber::Mode& m2)
// {
//   return (m1.fps == m2.fps &&
//           m1.depth_width == m2.depth_width &&
//           m1.depth_height == m2.depth_height &&
//           m1.color_width == m2.color_width &&
//           m1.color_height == m2.color_height);
// }

RealSenseGrabber::RealSenseGrabber (const std::string& device_id, bool strict)
: pcl::Grabber ()
, is_running_ (false)
// , confidence_threshold_ (6)
// , temporal_filtering_type_ (RealSense_None)
// , temporal_filtering_window_size_ (1)
// , mode_requested_ (mode)
, strict_ (strict)
{
  if (device_id == "")
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice ();
  else if (device_id[0] == '#')
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (device_id);

  point_cloud_signal_ = createSignal<sig_cb_real_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_real_sense_point_cloud_rgba> ();
}

RealSenseGrabber::~RealSenseGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_real_sense_point_cloud> ();
  disconnect_all_slots<sig_cb_real_sense_point_cloud_rgba> ();
}

void
RealSenseGrabber::start ()
{
  static bool first = true;

  if (!is_running_)
  {
    //need_xyz_ = num_slots<sig_cb_real_sense_point_cloud> () > 0;
    //need_xyzrgba_ = num_slots<sig_cb_real_sense_point_cloud_rgba> () > 0;
    if (!device_->getRSDevice ().is_streaming())
    {
      // Configure and start our device
      device_->getRSDevice ().enable_stream(rs::stream::depth, rs::preset::largest_image);
      device_->getRSDevice ().enable_stream(rs::stream::color, rs::preset::largest_image);
      device_->getRSDevice ().set_option(rs::option::r200_emitter_enabled, 1);
      //rs_apply_depth_control_preset((rs_device *)&device_->getRSDevice (), static_cast<int>(5));
      // device_->getRSDevice ().enable_stream(rs::stream::infrared, rs::preset::largest_image);
      // try { device_->getRSDevice ().enable_stream(rs::stream::infrared2, rs::preset::largest_image); } catch(...) {}
     // std::cerr << "Device configured" << std::endl;
      device_->getRSDevice ().start();
     // std::cerr << "After Device started" << std::endl;
      // selectMode ();
      // PXCCapture::Device::StreamProfileSet profile;
      // memset (&profile, 0, sizeof (profile));
      // profile.depth.frameRate.max = mode_selected_.fps;
      // profile.depth.frameRate.min = mode_selected_.fps;
      // profile.depth.imageInfo.width = mode_selected_.depth_width;
      // profile.depth.imageInfo.height = mode_selected_.depth_height;
      // profile.depth.imageInfo.format = PXCImage::PIXEL_FORMAT_DEPTH;
      // profile.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;
      // if (need_xyzrgba_)
      // {
      //   profile.color.frameRate.max = mode_selected_.fps;
      //   profile.color.frameRate.min = mode_selected_.fps;
      //   profile.color.imageInfo.width = mode_selected_.color_width;
      //   profile.color.imageInfo.height = mode_selected_.color_height;
      //   profile.color.imageInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
      //   profile.color.options = PXCCapture::Device::STREAM_OPTION_ANY;
      // }
      // device_->getRSDevice ().SetStreamProfileSet (&profile);
      // if (!device_->getRSDevice ().IsStreamProfileSetValid (&profile))
      //   THROW_IO_EXCEPTION ("Invalid stream profile for PXC device");
      if (first) {
        fps_mutex_.lock ();
      frequency_.reset ();
      fps_mutex_.unlock ();
      first = false;
      }
      
      //std::cerr << "After frequency_ reset" << std::endl;
      is_running_ = true;
      run();
      //thread_ = boost::thread (&RealSenseGrabber::run, this);
    }
  }
}

void
RealSenseGrabber::stop ()
{
  if (is_running_)
  {
    is_running_ = false;
    device_->getRSDevice ().stop();
    //thread_.join ();
  }
}

bool
RealSenseGrabber::isRunning () const
{
  return (is_running_);
}

float
RealSenseGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

// void
// pcl::RealSenseGrabber::setConfidenceThreshold (unsigned int threshold)
// {
//   if (threshold > 15)
//   {
//     PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] Attempted to set threshold outside valid range (0-15)");
//   }
//   else
//   {
//     confidence_threshold_ = threshold;
//     device_->getRSDevice ().SetDepthConfidenceThreshold (confidence_threshold_);
//   }
// }

// void
// pcl::RealSenseGrabber::enableTemporalFiltering (TemporalFilteringType type, size_t window_size)
// {
//   if (temporal_filtering_type_ != type ||
//      (type != RealSense_None && temporal_filtering_window_size_ != window_size))
//   {
//     temporal_filtering_type_ = type;
//     temporal_filtering_window_size_ = window_size;
//     if (is_running_)
//     {
//       stop ();
//       start ();
//     }
//   }
// }

// void
// pcl::RealSenseGrabber::disableTemporalFiltering ()
// {
//   enableTemporalFiltering (RealSense_None, 1);
// }

const std::string&
RealSenseGrabber::getDeviceSerialNumber () const
{
  return (device_->getSerialNumber ());
}

// std::vector<pcl::RealSenseGrabber::Mode>
// pcl::RealSenseGrabber::getAvailableModes (bool only_depth) const
// {
//   std::vector<Mode> modes;
//   PXCCapture::StreamType streams = only_depth
//     ? PXCCapture::STREAM_TYPE_DEPTH
//     : PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR;
//   for (int p = 0;; p++)
//   {
//     PXCCapture::Device::StreamProfileSet profiles = {};
//     if (device_->getPXCDevice ().QueryStreamProfileSet (streams, p, &profiles) == PXC_STATUS_NO_ERROR)
//     {
//       if (!only_depth && profiles.depth.frameRate.max != profiles.color.frameRate.max)
//         continue; // we need both streams to have the same framerate
//       Mode mode;
//       mode.fps = profiles.depth.frameRate.max;
//       mode.depth_width = profiles.depth.imageInfo.width;
//       mode.depth_height = profiles.depth.imageInfo.height;
//       mode.color_width = profiles.color.imageInfo.width;
//       mode.color_height = profiles.color.imageInfo.height;
//       bool duplicate = false;
//       for (size_t i = 0; i < modes.size (); ++i)
//         duplicate |= modes[i] == mode;
//       if (!duplicate)
//         modes.push_back (mode);
//     }
//     else
//     {
//       break;
//     }
//   }
//   return modes;
// }

// void
// pcl::RealSenseGrabber::setMode (const Mode& mode, bool strict)
// {
//   if (mode == mode_requested_ && strict == strict_)
//     return;
//   mode_requested_ = mode;
//   strict_ = strict;
//   if (is_running_)
//   {
//     stop ();
//     start ();
//   }
// }
// struct state { double yaw, pitch, lastX, lastY; bool ml; std::vector<rs::stream> tex_streams; int index; rs::device * dev; };
void
RealSenseGrabber::run ()
{
  int wait = 0;

  // std::cerr << "RUNNING" << std::endl;
  while (is_running_)
  {
    //pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    //std::cerr << "RUNNING" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
    // state app_state = {0, 0, 0, 0, false, {rs::stream::color, rs::stream::depth, rs::stream::infrared}, 0, &device_->getRSDevice ()};
    // if(device_->getRSDevice ().is_stream_enabled(rs::stream::infrared2)) app_state.tex_streams.push_back(rs::stream::infrared2);

    device_->getRSDevice ().wait_for_frames();
    //std::cerr << "AFTER wait_for_frames" << std::endl;
    auto depth_image = reinterpret_cast<const uint16_t *>(device_->getRSDevice ().get_frame_data(rs::stream::depth));
    auto color_image = reinterpret_cast<const uint8_t *>(device_->getRSDevice ().get_frame_data(rs::stream::color));
    
    wait++;
    if (wait < 10) continue;
    //std::cerr << "NEW FRAME" << std::endl;
    // Retrieve our images
    //const uint16_t * depth_image = (const uint16_t *)device_->getRSDevice ().get_frame_data(rs::stream::depth);

    // Retrieve camera parameters for mapping between depth and color
    //const float depth_scale = device_->getRSDevice ().get_depth_scale();
    const rs::intrinsics depth_intrin = device_->getRSDevice ().get_stream_intrinsics(rs::stream::depth);
    //rs::intrinsics depth_intrin = device_->getRSDevice ().get_stream_intrinsics(rs::stream::depth);
    const rs::extrinsics depth_to_color = device_->getRSDevice ().get_extrinsics(rs::stream::depth, rs::stream::color);
    const rs::intrinsics color_intrin = device_->getRSDevice ().get_stream_intrinsics(rs::stream::color);
    const float scale = device_->getRSDevice ().get_depth_scale();


    const int WIDTH = depth_intrin.width;
    const int HEIGHT = depth_intrin.height;
    //const int SIZE = WIDTH * HEIGHT;

    // std::cerr << "width == " << WIDTH << " -- height == " << HEIGHT << std::endl;
    // for (int i = 0; i < SIZE; ++i) {
    //       std::cout << "depth_image[" << i << "] is == " << depth_image[i] << std::endl;
    // }
    // break;
    //std::vector<rs::float3> vertices (SIZE);

    // pxcStatus status;
    // if (need_xyzrgba_)
    //   status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR, &sample);
    // else
    //   status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH, &sample);

    uint64_t timestamp = pcl::getTime () * 1.0e+6;

    // switch (status)
    // {
    // case PXC_STATUS_NO_ERROR:
    // {
      fps_mutex_.lock ();
      frequency_.event ();
      fps_mutex_.unlock ();
      /* We preform the following steps to convert received data into point clouds:
       *
       *   1. Push depth image to the depth buffer
       *   2. Pull filtered depth image from the depth buffer
       *   3. Project (filtered) depth image into 3D
       *   4. Fill XYZ point cloud with computed points
       *   5. Fill XYZRGBA point cloud with computed points
       *   7. Project color image into 3D
       *   6. Assign colors to points in XYZRGBA point cloud
       *
       * Steps 1-2 are skipped if temporal filtering is disabled.
       * Step 4 is skipped if there are no subscribers for XYZ clouds.
       * Steps 5-7 are skipped if there are no subscribers for XYZRGBA clouds. */

      // if (temporal_filtering_type_ != RealSense_None)
      // {
      //   PXCImage::ImageData data;
      //   sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
      //   std::vector<unsigned short> data_copy (SIZE);
      //   memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
      //   sample.depth->ReleaseAccess (&data);

      //   depth_buffer_->push (data_copy);

      //   sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
      //   unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
      //   for (size_t i = 0; i < SIZE; i++)
      //     d[i] = (*depth_buffer_)[i];
      //   sample.depth->ReleaseAccess (&data);
      // }

      //projection->QueryVertices (sample.depth, vertices.data ());

      // if (need_xyz_)
      // {
      //   xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
      //   xyz_cloud->header.stamp = timestamp;
      //   xyz_cloud->is_dense = false;
      //   for (int i = 0; i < SIZE; i++)
      //     convertPoint (vertices[i], xyz_cloud->points[i]);
      // }

      // if (need_xyzrgba_)
      // {
        // PXCImage::ImageData data;
        // PXCImage* mapped = projection->CreateColorImageMappedToDepth (sample.depth, sample.color);
        // mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
        // uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
        // if (need_xyz_)
        // {
        //   // We can fill XYZ coordinates more efficiently using pcl::copyPointCloud,
        //   // given that they were already computed for XYZ point cloud.
        //   xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
        //   pcl::copyPointCloud (*xyz_cloud, *xyzrgba_cloud);
        //   for (int i = 0; i < HEIGHT; i++)
        //   {
        //     pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];
        //     uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
        //     for (int j = 0; j < WIDTH; j++)
        //       memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
        //   }
        // }
        // else
        // {
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
          xyzrgba_cloud->header.stamp = timestamp;
          xyzrgba_cloud->is_dense = false;
          //int has_one_point = 0;
          // auto color_aligned_to_depth = reinterpret_cast<const rs::float3 *>(device_->getRSDevice ().get_frame_data(rs::stream::color_aligned_to_depth));
          // auto depth_aligned_to_color = reinterpret_cast<const rs::float3 *>(device_->getRSDevice ().get_frame_data(rs::stream::depth_aligned_to_color));
          // auto depth_aligned_to_rectified_color = reinterpret_cast<const rs::float3 *>(device_->getRSDevice ().get_frame_data(rs::stream::depth_aligned_to_rectified_color));
          for (int dy = 0; dy < HEIGHT; dy++)
          {
            //rs::float3* vertices_row = &vertices[i * WIDTH];
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[dy * WIDTH];
            // uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
            for (int dx = 0; dx < WIDTH; dx++)
            {
              // Retrieve the 16-bit depth value and map it into a depth in meters
              uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
              float depth_in_meters = depth_value * scale;
              //std::cerr << "Points z is ==  " << points->z << std::endl;
              // Skip over pixels with a depth value of zero, which is used to indicate no data
              if (depth_value == 0) continue;
              //++has_one_point;
              //{
                //std::cout << "FOUND Z POINT" << std::endl;
                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                convertPoint (depth_point, cloud_row[dx]);
                //printf("Color is == %016X\n", *color);
                uint8_t r = 255;
                uint8_t g = 255;
                uint8_t b = 255;
                // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
                {
                    r = 255;
                    g = 255;
                    b = 255;
                }
                else
                {
                    const uint8_t *c = color_image + (cy * color_intrin.width + cx) * 3;
                    //std::cerr << "Color found == "<< std::hex << (int)c << std::endl;
                    r = c[0];
                    g = c[1];
                    b = c[2];
                }
                //std::cerr << "Color point x == " << std::hex << (int)r << " -- y == " << (int)g << " -- z == " << (int)b << std::endl;
                // std::cerr << "depth_aligned_to_rectified_color point x == " << depth_aligned_to_rectified_color->x << " -- y == " << depth_aligned_to_rectified_color->y << " -- z == " << depth_aligned_to_rectified_color->z << std::endl;
                // std::cerr << "depth_aligned_to_color point x == " << depth_aligned_to_color->x << " -- y == " << depth_aligned_to_color->y << " -- z == " << depth_aligned_to_color->z << std::endl;
                // std::cerr << "color_aligned_to_depth point x == " << color_aligned_to_depth->x << " -- y == " << color_aligned_to_depth->y << " -- z == " << color_aligned_to_depth->z << std::endl;
                cloud_row[dx].r = r;
                cloud_row[dx].g = g;
                cloud_row[dx].b = b;
                //std::cerr << "Point cloud got == " << cloud_row[dx] << std::endl;
              //}
              //++color;
              //++points;
              // ++depth_aligned_to_rectified_color;
              // ++depth_aligned_to_color;
              // ++color_aligned_to_depth;
              //memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
            }
          }
        //}
        // mapped->ReleaseAccess (&data);
        // mapped->Release ();
      //}

      //if (need_xyzrgba_)
          //std::cerr << "Sending signal" << std::endl;
        //   std::cout << "IN REALSENSE PointCloud is == " << std::endl;
        // for (size_t i = 0; i < xyzrgba_cloud->points.size (); ++i)
        //   std::cout << "    " << xyzrgba_cloud->points[i].x
        //             << " "    << xyzrgba_cloud->points[i].y
        //             << " "    << xyzrgba_cloud->points[i].z << std::endl;
        //std::cerr << "IN REALSENSE End point cloud empty is " << has_one_point<< std::endl;
        // if (has_one_point > 10000)
        //   {
            //std::cerr << "Sending End point cloud empty is " << has_one_point<< std::endl;
            point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
            return;
          //}
      //if (need_xyz_)
       // point_cloud_signal_->operator () (xyz_cloud);
      //break;
    //}
    // case PXC_STATUS_DEVICE_LOST:
    //   THROW_IO_EXCEPTION ("failed to read data stream from PXC device: device lost");
    // case PXC_STATUS_ALLOC_FAILED:
    //   THROW_IO_EXCEPTION ("failed to read data stream from PXC device: alloc failed");
    // }
    // sample.ReleaseImages ();
  }
  //projection->Release ();
  RealSenseDevice::reset (device_);
}

// float
// pcl::RealSenseGrabber::computeModeScore (const Mode& mode)
// {
//   const float FPS_WEIGHT = 100000;
//   const float DEPTH_WEIGHT = 1000;
//   const float COLOR_WEIGHT = 1;
//   int f = mode.fps - mode_requested_.fps;
//   int dw = mode.depth_width - mode_requested_.depth_width;
//   int dh = mode.depth_height - mode_requested_.depth_height;
//   int cw = mode.color_width - mode_requested_.color_width;
//   int ch = mode.color_height - mode_requested_.color_height;
//   float penalty;
//   penalty  = std::abs (FPS_WEIGHT * f * (mode_requested_.fps != 0));
//   penalty += std::abs (DEPTH_WEIGHT * dw * (mode_requested_.depth_width != 0));
//   penalty += std::abs (DEPTH_WEIGHT * dh * (mode_requested_.depth_height != 0));
//   penalty += std::abs (COLOR_WEIGHT * cw * (mode_requested_.color_width != 0 && need_xyzrgba_));
//   penalty += std::abs (COLOR_WEIGHT * ch * (mode_requested_.color_height != 0 && need_xyzrgba_));
//   return penalty;
// }

// void
// pcl::RealSenseGrabber::selectMode ()
// {
//   if (mode_requested_ == Mode ())
//     mode_requested_ = Mode (30, 640, 480, 640, 480);
//   float best_score = std::numeric_limits<float>::max ();
//   std::vector<Mode> modes = getAvailableModes (!need_xyzrgba_);
//   for (size_t i = 0; i < modes.size (); ++i)
//   {
//     Mode mode = modes[i];
//     float score = computeModeScore (mode);
//     if (score < best_score)
//     {
//       best_score = score;
//       mode_selected_ = mode;
//     }
//   }
//   if (strict_ && best_score > 0)
//     THROW_IO_EXCEPTION ("PXC device does not support requested mode");
// }

// void
// pcl::RealSenseGrabber::createDepthBuffer ()
// {
//   size_t size = mode_selected_.depth_width * mode_selected_.depth_height;
//   switch (temporal_filtering_type_)
//   {
//   case RealSense_None:
//   {
//     depth_buffer_.reset (new pcl::io::SingleBuffer<unsigned short> (size));
//     break;
//   }
//   case RealSense_Median:
//   {
//     depth_buffer_.reset (new pcl::io::MedianBuffer<unsigned short> (size, temporal_filtering_window_size_));
//     break;
//   }
//   case RealSense_Average:
//   {
//     depth_buffer_.reset (new pcl::io::AverageBuffer<unsigned short> (size, temporal_filtering_window_size_));
//     break;
//   }
//   }
// }

