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

#include "h264_image_transport/h264_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <vector>
#include <cstdio> //for memcpy

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace h264_image_transport {

H264Publisher::H264Publisher()
{
 
}

H264Publisher::~H264Publisher()
{
}

void H264Publisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                    const image_transport::SubscriberStatusCallback  &user_connect_cb,
                                    const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                                    const ros::VoidPtr &tracked_object, bool latch)
{
  // queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
  queue_size += 1;
  // Latching doesn't make a lot of sense with this transport. Could try to save the last keyframe,
  // but do you then send all following delta frames too?
  latch = false;
  typedef image_transport::SimplePublisherPlugin<h264_image_transport::Packet> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&H264Publisher::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}


void H264Publisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  // Send the header packets to new subscribers
  for (unsigned int i = 0; i < stream_header_.size(); i++) {
    pub.publish(stream_header_[i]);
  }
}


void H264Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  
}


} //namespace h264_image_transport
