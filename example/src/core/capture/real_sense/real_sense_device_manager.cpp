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

#include <pcl/io/io_exception.h>

#include <capture/real_sense/real_sense_device_manager.h>

boost::mutex RealSenseDeviceManager::mutex_;


/** Helper function to release a RealSense resource.
    Useful as a deleter for a shared pointer holding RealSense resource. */
template <typename T> void
releaseResource (T* resource)
{
  if (resource)
  {
    //resource->Release ();
    resource = 0;
  }
}

template <typename T> boost::shared_ptr<T>
makeSharedPtr (T* resource)
{
  return boost::shared_ptr<T> (resource, releaseResource<T>);
}

// boost::shared_ptr<PXCSession>
// createPXCSession ()
// {
//   PXCSession* s = PXCSession::CreateInstance ();
//   if (!s)
//     THROW_IO_EXCEPTION ("failed to create RealSense session");
//   return makePXCSharedPtr (s);
// }

boost::shared_ptr<rs::context>
createCaptureManager ()
{
  rs::context *ctx = new rs::context();
  return makeSharedPtr (ctx);
}

// boost::shared_ptr<PXCCapture>
// createPXCCapture (PXCSession& session, pxcUID iuid)
// {
//   PXCCapture* c;
//   if (session.CreateImpl (iuid, &c) < PXC_STATUS_NO_ERROR)
//     THROW_IO_EXCEPTION ("unable to create RealSense capture");
//   return makePXCSharedPtr (c);
// }

boost::shared_ptr<rs::device>
createCaptureDevice (rs::context &ctx, int device_number)
{
  rs::device* d;
  d = ctx.get_device(device_number);
  if (!d)
    pcl::io::THROW_IO_EXCEPTION ("unable to create RealSense capture device");
  return makeSharedPtr (d);
}

/** Utility function to convert RealSense-style strings (which happen to
  * consist of 2-byte chars) into standard library strings. */
// std::string
// toString (const pxcCHAR* pxc_string, size_t max_length)
// {
//   size_t i = 0;
//   while (i + 1 < max_length && pxc_string[i])
//     ++i;
//   std::string out (i + 1, '\0');
//   size_t j = 0;
//   while (j < i)
//     out[j] = pxc_string[j++];
//   return out;
// }

RealSenseDeviceManager::RealSenseDeviceManager ()
{
  populateDeviceList ();
}

RealSenseDeviceManager::~RealSenseDeviceManager ()
{
}

RealSenseDevice::Ptr
RealSenseDeviceManager::captureDevice ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (device_list_.size () == 0)
    pcl::io::THROW_IO_EXCEPTION ("no connected devices");
  for (size_t i = 0; i < device_list_.size (); ++i)
    if (!device_list_[i].isCaptured ())
      return (capture (device_list_[i]));
  pcl::io::THROW_IO_EXCEPTION ("all connected devices are captured by other grabbers");
  return (RealSenseDevice::Ptr ());  // never reached, needed just to silence -Wreturn-type warning
}

RealSenseDevice::Ptr
RealSenseDeviceManager::captureDevice (size_t index)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (index >= device_list_.size ())
    pcl::io::THROW_IO_EXCEPTION ("device with index %i is not connected", index + 1);
  if (device_list_[index].isCaptured ())
    pcl::io::THROW_IO_EXCEPTION ("device with index %i is captured by another grabber", index + 1);
  return (capture (device_list_[index]));
}

RealSenseDevice::Ptr
RealSenseDeviceManager::captureDevice (const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  for (size_t i = 0; i < device_list_.size (); ++i)
  {
    if (device_list_[i].serial == sn)
    {
      if (device_list_[i].isCaptured ())
        pcl::io::THROW_IO_EXCEPTION ("device with serial number %s is captured by another grabber", sn.c_str ());
      return (capture (device_list_[i]));
    }
  }
  pcl::io::THROW_IO_EXCEPTION ("device with serial number %s is not connected", sn.c_str ());
  return (RealSenseDevice::Ptr ());  // never reached, needed just to silence -Wreturn-type warning
}

void
RealSenseDeviceManager::populateDeviceList ()
{
  device_list_.clear ();
  capture_manager_ = createCaptureManager();

  // Module description to match
  // PXCSession::ImplDesc module_desc = {};
  // module_desc.group = PXCSession::IMPL_GROUP_SENSOR;
  // module_desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
  if (capture_manager_->get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

  for (int m = 0; m < capture_manager_->get_device_count(); m++)
  {
    boost::shared_ptr<rs::device> device = createCaptureDevice(*capture_manager_, m);
       
    device_list_.push_back (DeviceInfo ());
    device_list_.back ().serial = device->get_serial();
    device_list_.back ().name = device->get_name();
    device_list_.back ().firmware_version = device->get_firmware_version();
    device_list_.back ().idx = m;
    //device_list_.back ().device_ptr = device;

  }
}

RealSenseDevice::Ptr
RealSenseDeviceManager::capture (DeviceInfo& device_info)
{
  // This is called from public captureDevice() functions and should already be
  // under scoped lock
  if (!device_info.device_ptr.expired ())
  {
    return device_info.device_ptr.lock ();
  }
  else
  {
    RealSenseDevice::Ptr device (new RealSenseDevice (device_info.serial));
    device->device_ = createCaptureDevice (*capture_manager_, device_info.idx);
    
    device_info.device_ptr = device;
    return device;
  }
}

