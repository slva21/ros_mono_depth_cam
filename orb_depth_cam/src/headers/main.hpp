// SYSTEMS DEPS
#include <iostream>
#include <chrono>
#include <vector>

// ROS DEPENENCIES
#include <thread>
#include <ros/ros.h>
#include </home/vslam/catkin_ws/src/vision_opencv/cv_bridge/include/cv_bridge/cv_bridge.h>

#include <pthread.h>

// SCR DEPS
#include "./orb.hpp"
#include "./lms_solver.hpp"
#include "../ros/image_publisher.hpp"
#include "../ros/image_subscriber.hpp"
#include "./ego_motion.hpp"
#include "./midas.hpp"
std::vector<std::thread> publisher_instances;

using namespace cv;
