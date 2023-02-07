#pragma once
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include </home/vslam/catkin_ws/src/vision_opencv/cv_bridge/include/cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

class ImageSubcriber
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  cv::Mat ros_img_src; // Subscriber Image 
  unsigned int ros_frame_count; // Subscriber Image count

  ImageSubcriber(cv::Mat **src, unsigned int **p_frame_count) // the location of a src pointer is passed
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
                               &ImageSubcriber::imageCb, this);

    *src = &ros_img_src; // assign the value of that pointer at passed location to class vairable
    *p_frame_count = &ros_frame_count;
    ros_frame_count=0;
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // update variable
    ros_img_src = cv_ptr->image;

    ros_frame_count++;

    if (ros_frame_count == UINT_MAX)
      ros_frame_count = 0;
  }
};