#pragma once
#include <iostream>

#include <ros/ros.h>
#include <thread>
#include <pthread.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include </home/vslam/catkin_ws/src/vision_opencv/cv_bridge/include/cv_bridge/cv_bridge.h>

class ImagePublisher
{
private: 
    ros::NodeHandle nh;
    cv::Mat *img;                 // source to published image
    unsigned int ros_frame_count; // Publisher Image count

    image_transport::ImageTransport it;
    image_transport::Publisher pub;

    ros::Rate loop_rate;

public:
    void publishMessage()
    {
        while (nh.ok()) // Maby add check if image has changed too ..
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img).toImageMsg();
            pub.publish(msg);
            loop_rate.sleep();
        }
    }

    std::thread imagePublisher_thread()
    {
        return std::thread([=]
                           { publishMessage(); });
    }

public:
    ImagePublisher(std::string topic, cv::Mat &_img, int rate);
};

ImagePublisher::ImagePublisher(std::string topic, cv::Mat &_img, int rate) : it(nh) , loop_rate(rate)
{
    img = &_img;
    pub = it.advertise(topic, 1);
}



// ImagePublisher *img_publisher = new ImagePublisher(src_img, loop_rate); // create instances
// std::thread t = img_publisher->imagePublisher_thread(); // Start thread
