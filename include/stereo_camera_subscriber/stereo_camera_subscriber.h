/*
 * visual_odometer_nodelet.h
 *
 * This is an extension and partial copy of the ros CameraSubscriber class
 *
 * @date 2011-11-24
 * @author Volker Nannen
 */

#ifndef STEREO_CAMERA_SUBSCRIBER_H
#define STEREO_CAMERA_SUBSCRIBER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "image_transport/transport_hints.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace stereo_camera_subscriber {

class StereoCameraSubscriber
{
public:
  typedef boost::function<void ( const sensor_msgs::ImageConstPtr &,
                                 const sensor_msgs::CameraInfoConstPtr &,
                                 const sensor_msgs::ImageConstPtr &,
                                 const sensor_msgs::CameraInfoConstPtr & )>
  Callback;

  StereoCameraSubscriber() {}

  std::string getLeftTopic() const;

  std::string getLeftInfoTopic() const;

  std::string getRightTopic() const;

  std::string getRightInfoTopic() const;

  uint32_t getLeftNumPublishers() const;

  uint32_t getRightNumPublishers() const;

  std::string getLeftTransport() const;

  std::string getRightTransport() const;

  void shutdown();

  operator void *() const;
  bool operator< ( const StereoCameraSubscriber & rhs ) const {
    return impl_ <  rhs.impl_;
  }
  bool operator!=( const StereoCameraSubscriber & rhs ) const {
    return impl_ != rhs.impl_;
  }
  bool operator==( const StereoCameraSubscriber & rhs ) const {
    return impl_ == rhs.impl_;
  }

  StereoCameraSubscriber( image_transport::ImageTransport & image_it,
                          ros::NodeHandle & info_nh,
                          const std::string & base_topic_left,
                          const std::string & base_topic_right,
                          uint32_t queue_size,
                          const Callback & callback,
                          const ros::VoidPtr & tracked_object = ros::VoidPtr(),
                          const image_transport::TransportHints &
                          transport_hints = image_transport::TransportHints() );

private:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl>   ImplWPtr;

  ImplPtr impl_;

  friend class image_transport::ImageTransport;
};

} //namespace image_transport

 #endif