/*
 * visual_odometer_nodelet.cpp
 *
 * This is an extension and partial copy of the ros StereoCameraSubscriber class
 *
 * @date 2011-11-24
 * @author Volker Nannen
 */

//#include <stdio.h>
//#include <stdlib.h>
//
//#include <cstdlib>
////#include <iostream>
//#include <cstring>
//#include <fstream>

#include "stereo_camera_subscriber/stereo_camera_subscriber.h"
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace stereo_camera_subscriber {

void increment( int * value ) {
  ++( *value );
}
  
void report( const boost::shared_ptr<const sensor_msgs::Image>& image ) {
  std::cout "Image" << std::endl
  << "    Seq:     " << image->header.seq << std::endl 
  << "    Stamp:   " << image->header.stamp << std::endl 
  << "    FrameID: " << image->header.frame_id << std::endl;
}    

struct StereoCameraSubscriber::Impl {

  Impl( uint32_t queue_size )
    : sync_( queue_size ),
    unsubscribed_( false ),
    image_received_left_( 0 ),
    info_received_left_( 0 ),
    image_received_right_( 0 ),
    info_received_right_( 0 ),
    all_received_( 0 ),
    debug_( false )
  {}

  ~Impl() {
    shutdown();
  }

  bool isValid() const {
    return !unsubscribed_;
  }

  void shutdown() {
    if ( !unsubscribed_ ) {
      unsubscribed_ = true;
      image_sub_left_.unsubscribe();
      info_sub_left_.unsubscribe();
      image_sub_right_.unsubscribe();
      info_sub_right_.unsubscribe();
    }
  }

  void checkImagesSynchronized() {
    int threshold = 3 * all_received_;
    if ( debug_ or 
         ( image_received_left_ > threshold or
           info_received_left_ > threshold or
           image_received_right_ > threshold or
           info_received_right_ > threshold ) ) {
      ROS_WARN( "[image_transport] Topics '%s', '%s', '%s' and '%s' "
                "do not appear to be synchronized. "
                "In the last 10s:\n"
                "\tleft  Image messages received:      %d\n"
                "\tleft  CameraInfo messages received: %d\n"
                "\tright Image messages received:      %d\n"
                "\tright CameraInfo messages received: %d\n"
                "\tSynchronized couples:               %d",
                image_sub_left_.getTopic().c_str(),
                info_sub_left_.getTopic().c_str(),
                image_sub_right_.getTopic().c_str(),
                info_sub_right_.getTopic().c_str(),
                image_received_left_,
                info_received_left_,
                image_received_right_,
                info_received_right_,
                all_received_ );
    }
    image_received_left_ = 0;
    info_received_left_ = 0;
    image_received_right_ = 0;
    info_received_right_ = 0;
    all_received_ = 0;
  }

  image_transport::SubscriberFilter image_sub_left_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_left_;
  image_transport::SubscriberFilter image_sub_right_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_right_;

  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    sensor_msgs::CameraInfo,
                                    sensor_msgs::Image,
                                    sensor_msgs::CameraInfo> sync_;

//  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
//                                                    sensor_msgs::CameraInfo,
//                                                    sensor_msgs::Image,
//                                                    sensor_msgs::CameraInfo>
//  ExactStereoPolicy;
//  message_filters::Synchronizer<ExactStereoPolicy> sync_;

  bool unsubscribed_;
  // For detecting when the topics aren't synchronized
  ros::WallTimer check_synced_timer_;
  int image_received_left_;
  int info_received_left_;
  int image_received_right_;
  int info_received_right_;
  int all_received_;
  bool debug_;
};

StereoCameraSubscriber::
StereoCameraSubscriber( image_transport::ImageTransport & image_it,
                        ros::NodeHandle & info_nh,
                        const std::string & base_topic_left,
                        const std::string & base_topic_right,
                        uint32_t queue_size,
                        const Callback & callback,
                        const ros::VoidPtr & tracked_object,
                        const image_transport::TransportHints &
                        transport_hints )
  : impl_( new Impl( queue_size ) ) {
  // Must explicitly remap the image topic since we then do some string 
  // manipulation on it to figure out the sibling camera_info topic.
  std::string image_topic_left =
    info_nh.resolveName( base_topic_left );
  std::string info_topic_left =
    image_transport::getCameraInfoTopic( image_topic_left );
  std::string image_topic_right =
    info_nh.resolveName( base_topic_right );
  std::string info_topic_right =
    image_transport::getCameraInfoTopic( image_topic_right );
  impl_->image_sub_left_.subscribe( image_it,
                                    image_topic_left,
                                    queue_size,
                                    transport_hints );
  impl_->info_sub_left_.subscribe( info_nh,
                                   info_topic_left,
                                   queue_size,
                                   transport_hints.getRosHints() );
  impl_->image_sub_right_.subscribe( image_it,
                                     image_topic_right,
                                     queue_size,
                                     transport_hints );
  impl_->info_sub_right_.subscribe( info_nh,
                                    info_topic_right,
                                    queue_size,
                                    transport_hints.getRosHints() );
  impl_->sync_.connectInput( impl_->image_sub_left_,
                             impl_->info_sub_left_,
                             impl_->image_sub_right_,
                             impl_->info_sub_right_ );
  // need for Boost.Bind here is kind of broken
  impl_->sync_.registerCallback( boost::bind( callback, _1, _2, _3, _4 ) );

  // Complain every 10s if it appears that the image and info topics 
  // are not synchronized
  impl_->image_sub_left_.registerCallback( boost::bind( report, _1 ) );
//  impl_->info_sub_left_.
//  registerCallback( boost::bind( report ) );
  impl_->image_sub_right_.registerCallback( boost::bind( report, _1 ) );
//  impl_->info_sub_right_.
//  registerCallback( boost::bind( report ) );
    
  impl_->image_sub_left_.
  registerCallback( boost::bind( increment, &impl_->image_received_left_ ) );
  impl_->info_sub_left_.
  registerCallback( boost::bind( increment, &impl_->info_received_left_ ) );
  impl_->image_sub_right_.
  registerCallback( boost::bind( increment, &impl_->image_received_right_ ) );
  impl_->info_sub_right_.
  registerCallback( boost::bind( increment, &impl_->info_received_right_ ) );
  impl_->sync_.
  registerCallback( boost::bind( increment, &impl_->all_received_ ) );
  impl_->check_synced_timer_ =
    info_nh.createWallTimer( ros::WallDuration( 10.0 ),
                             boost::bind( &Impl::checkImagesSynchronized,
                                          impl_ ) );
}

std::string StereoCameraSubscriber::getLeftTopic() const {
  if ( impl_ ) return impl_->image_sub_left_.getTopic();
  return std::string();
}

std::string StereoCameraSubscriber::getLeftInfoTopic() const {
  if ( impl_ ) return impl_->info_sub_left_.getTopic();
  return std::string();
}

std::string StereoCameraSubscriber::getRightTopic() const {
  if ( impl_ ) return impl_->image_sub_right_.getTopic();
  return std::string();
}

std::string StereoCameraSubscriber::getRightInfoTopic() const {
  if ( impl_ ) return impl_->info_sub_right_.getTopic();
  return std::string();
}

uint32_t StereoCameraSubscriber::getLeftNumPublishers() const {
  //if (impl_) return std::max(impl_->image_sub_.getNumPublishers(),
  //                           impl_->info_sub_.getNumPublishers());
  if ( impl_ ) return impl_->image_sub_left_.getNumPublishers();
  return 0;
}

uint32_t StereoCameraSubscriber::getRightNumPublishers() const {
  //if (impl_) return std::max(impl_->image_sub_.getNumPublishers(),
  //                           impl_->info_sub_.getNumPublishers());
  if ( impl_ ) return impl_->image_sub_right_.getNumPublishers();
  return 0;
}

std::string StereoCameraSubscriber::getLeftTransport() const {
  if ( impl_ ) return impl_->image_sub_left_.getTransport();
  return std::string();
}

std::string StereoCameraSubscriber::getRightTransport() const {
  if ( impl_ ) return impl_->image_sub_right_.getTransport();
  return std::string();
}

void StereoCameraSubscriber::shutdown() {
  if ( impl_ ) impl_->shutdown();
}

StereoCameraSubscriber::operator void *() const {
  return ( impl_ && impl_->isValid() ) ? (void *)1 : (void *)0;
}

bool StereoCameraSubscriber::debug( bool on ) {
  return ( impl_->debug_ = on );  
}
  
} //namespace image_transport
