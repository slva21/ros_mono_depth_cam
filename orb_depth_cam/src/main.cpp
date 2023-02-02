#include "./headers/main.hpp"

using namespace std::chrono;
using namespace cv;

cv::Mat img1;
cv::Mat img2;

orb feature_method(img1, img2, true);

int main(int argc, char **argv)
{

    VideoCapture cap("v4l2src device=/dev/video0 ! video/x-raw,width=640, height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=(string)BGR ! appsink", CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cout << "VideoCapture" << std::endl;
        exit(-1);
    }

    while (true)
    {
        // Load the two images
        // img1 = cv::imread("/home/vslam/catkin_ws/src/orb_depth_cam/images/1.png");
        // img2 = cv::imread("/home/vslam/catkin_ws/src/orb_depth_cam/images/2.png");
        cap.read(img1);
        cap.read(img2);

        feature_method.compute();

        // Draw the matches on an image
        cv::Mat img_matches;
        cv::drawMatches(img1, feature_method.keypoints1, img2, feature_method.keypoints2, feature_method.matches, img_matches);

        // Display the result
        cv::imshow("Matches", img_matches);

        if (waitKey(1) == 's')
                break;
        
    }
   
    return 0;
}

// int main(int argc, char **argv)
// {

//     // Load the two images
//     cv::Mat img1 = cv::imread("/home/vslam/catkin_ws/src/orb_depth_cam/images/1.png");
//     cv::Mat img2 = cv::imread("/home/vslam/catkin_ws/src/orb_depth_cam/images/2.png");

//     // Create a brute-force matcher object
//     cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

//     // Define Discriptors and keypoints vectors
//     cv::Mat descriptors1, descriptors2;
//     std::vector<cv::KeyPoint> keypoints1, keypoints2;

//     if(ORB_GPU == true){
//         // Convert the images to grayscale on CPU
//         cv::Mat greyMat1, greyMat2;
//         cv::cvtColor(img1, greyMat1, cv::COLOR_BGR2GRAY);
//         cv::cvtColor(img2, greyMat2, cv::COLOR_BGR2GRAY);

//         // Define THE  ORB object
//         cv::Ptr<cv::ORB> orb = cv::ORB::create();

//         // Detect ORB features in the two images
//         orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
//         orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);
//     }
//     else{
//         // Convert the images to grayscale on GPU
//         cv::cuda::GpuMat gpuImg1, gpuImg2;

//         gpuImg1.upload(img1);
//         gpuImg2.upload(img2);

//         // Convert the images to grayscale
//         cv::cuda::GpuMat greyMat1, greyMat2;
//         cv::cuda::cvtColor(gpuImg1, greyMat1, cv::COLOR_BGR2GRAY);
//         cv::cuda::cvtColor(gpuImg2, greyMat2, cv::COLOR_BGR2GRAY);

//         // Define the ORB extractor
//         cv::Ptr<cv::cuda::ORB> orb = cv::cuda::ORB::create();

//         // Compute the keypoints and descriptors for both images
//         cv::cuda::GpuMat descriptors1GPU, descriptors2GPU;

//         orb->detectAndCompute(greyMat1, cv::cuda::GpuMat(), keypoints1, descriptors1GPU);
//         orb->detectAndCompute(greyMat2, cv::cuda::GpuMat(), keypoints2, descriptors2GPU);

//         // Convert the keypoints and descriptors from GPU to CPU memory

//         cv::cuda::GpuMat(descriptors1GPU).download(descriptors1);
//         cv::cuda::GpuMat(descriptors2GPU).download(descriptors2);
//     }

//     // Match the two sets of descriptors
//     std::vector<cv::DMatch> matches;
//     matcher->match(descriptors1, descriptors2, matches);

//     // Draw the matches on an image
//     // cv::Mat img_matches;
//     // cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);

//     // Display the result
//     // cv::imshow("Matches", img_matches);
//     // cv::waitKey(0);

//     return 0;
// }
