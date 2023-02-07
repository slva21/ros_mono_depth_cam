#include "./headers/main.hpp"

using namespace std::chrono;
using namespace cv;

cv::Mat img1;
cv::Mat img2;

orb feature_method(img1, img2);

// LMS lms;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "orb_depth_cam"); // Create node

    cv::Mat src_img;
    int loop_rate = 10;
    ImagePublisher *p = new ImagePublisher("/camera/image", src_img, loop_rate); // create instances

    // publisher_instances.push_back(p->imagePublisher_thread());  // Start thread

    // publisher_instances[0].join();

    VideoCapture cap("v4l2src device=/dev/video0 ! video/x-raw,width=640, height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=(string)BGR ! appsink", CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cout << "VideoCapture" << std::endl;
        exit(-1);
    }

    MIDAS::init();


    while (true)
    {
        // Load the two images
        // img1 = cv::imread("/home/vslam/catkin_ws/src/orb_depth_cam/images/1.png");
        // img2 = cv::imread("/home/vslam/catkin_ws/src/orb_depth_cam/images/2.png");

    
        auto t1 = getTickCount();

        cap.read(img1);
        cap.read(img2);

        auto t2 = getTickCount();
        auto delta = (t2 - t1) / getTickFrequency(); // time in seconds

        cv::Mat relative_depth = MIDAS::compute(img2);

        //  Find and match feature points using ORB
        feature_method.compute();

        // calculate the ground_truth points using Ego Motion
        std::vector<cv::KeyPoint> ground_truth_points = EGO_MOTION::evaluate_groundtruths(feature_method.matches, feature_method.keypoints1, feature_method.keypoints2, delta, 0.00367);

        if (ground_truth_points.size() > 1)
        {
            // Optimise scale_factor using Levenberg-Marquardt algorithm
            double scale = LMS_OPTIMSER::optimizeScale(relative_depth, ground_truth_points);

            std::cout << scale << std::endl;
        }

        // Draw the matches on an image
        cv::Mat img_matches;
        cv::drawMatches(img1, feature_method.keypoints1, img2, feature_method.keypoints2, feature_method.matches, img_matches);

        // Display the result
        cv::imshow("Matches", img_matches);
        cv::imshow("Relative Map", relative_depth);

        if (waitKey(1) == 's')
            break;
    }

    return 0;
}
