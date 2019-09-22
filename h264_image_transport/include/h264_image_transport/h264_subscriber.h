/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 20012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <image_transport/simple_subscriber_plugin.h>
#include <dynamic_reconfigure/server.h>
#include <h264_image_transport/H264SubscriberConfig.h>
#include <h264_image_transport/Packet.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace h264_image_transport {

class H264Subscriber : public image_transport::SimpleSubscriberPlugin<h264_image_transport::Packet>
{
public:
  H264Subscriber();
  virtual ~H264Subscriber();

  virtual std::string getTransportName() const { return "h264"; }

protected:
  // Overridden to bump queue_size, otherwise we might lose headers
  // Overridden to tweak arguments and set up reconfigure server
  virtual void subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                             const Callback &callback, const ros::VoidPtr &tracked_object,
                             const image_transport::TransportHints &transport_hints);
  
  // The function that does the actual decompression and calls a user supplied callback with the resulting image
  virtual void internalCallback(const h264_image_transport::PacketConstPtr &msg, const Callback& user_cb);

  // Dynamic reconfigure support
  typedef h264_image_transport::H264SubscriberConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  int pplevel_; // Post-processing level

  void configCb(Config& config, uint32_t level);

  // Utility functions
  int updatePostProcessingLevel(int level);

  bool received_header_;
  bool received_keyframe_;
  sensor_msgs::ImagePtr latest_image_;
};

} //namespace h264_image_transport
