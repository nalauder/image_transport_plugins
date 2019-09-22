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

#include "h264_image_transport/h264_subscriber.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/scoped_array.hpp>
#include <vector>

using namespace std;

namespace h264_image_transport {

H264Subscriber::H264Subscriber()
  : pplevel_(0),
    received_header_(false),
    received_keyframe_(false)
{
}

H264Subscriber::~H264Subscriber()
{
}

void H264Subscriber::subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                     const Callback &callback, const ros::VoidPtr &tracked_object,
                                     const image_transport::TransportHints &transport_hints)
{
  queue_size += 1;
  typedef image_transport::SimpleSubscriberPlugin<h264_image_transport::Packet> Base;
  Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&H264Subscriber::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void H264Subscriber::internalCallback(const h264_image_transport::PacketConstPtr& message, const Callback& callback)
{

}

} //namespace h264_image_transport
