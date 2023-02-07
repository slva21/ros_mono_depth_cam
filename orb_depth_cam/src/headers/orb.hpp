#include <iostream>

// NOTE
// It is possible to supply your own keypoints to orb algorithm. Could be an oportunity to make sure all segmnetations of the image are covered
// https://www.ccoderun.ca/programming/doxygen/opencv_3.2.0/classcv_1_1cuda_1_1Feature2DAsync.html

// OPENCV 4.5.5 DEPS
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
// -- OPENCV CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>

class orb
{
private:
    cv::Mat *img1;
    cv::Mat *img2;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    bool isGPU;

public:
    // DMatch.distance - Distance between descriptors. The lower, the better it is.
    // DMatch.trainIdx - Index of the descriptor in train descriptors(1st image)
    // DMatch.queryIdx - Index of the descriptor in query descriptors(2nd image)
    // DMatch.imgIdx - Index of the train image.
    
    std::vector<cv::DMatch> matches;
    // Define Discriptors and keypoints vectors
    cv::Mat descriptors1, descriptors2;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

public:
    int compute()
    {
        if (isGPU)
        {
            // Convert the images to grayscale on GPU
            cv::cuda::GpuMat gpuImg1, gpuImg2;

            gpuImg1.upload(*img1);
            gpuImg2.upload(*img2);

            // Convert the images to grayscale
            cv::cuda::GpuMat greyMat1, greyMat2;
            cv::cuda::cvtColor(gpuImg1, greyMat1, cv::COLOR_BGR2GRAY);
            cv::cuda::cvtColor(gpuImg2, greyMat2, cv::COLOR_BGR2GRAY);

            // Define the ORB extractor
            cv::Ptr<cv::cuda::ORB> orb = cv::cuda::ORB::create();

            // Compute the keypoints and descriptors for both images
            cv::cuda::GpuMat descriptors1GPU, descriptors2GPU;

            orb->detectAndCompute(greyMat1, cv::cuda::GpuMat(), keypoints1, descriptors1GPU);
            orb->detectAndCompute(greyMat2, cv::cuda::GpuMat(), keypoints2, descriptors2GPU);

            // Convert the keypoints and descriptors from GPU to CPU memory

            cv::cuda::GpuMat(descriptors1GPU).download(descriptors1);
            cv::cuda::GpuMat(descriptors2GPU).download(descriptors2);
        }
        else
        {
            // Convert the images to grayscale on CPU
            cv::Mat greyMat1, greyMat2;
            cv::cvtColor(*img1, greyMat1, cv::COLOR_BGR2GRAY);
            cv::cvtColor(*img2, greyMat2, cv::COLOR_BGR2GRAY);

            // Define THE  ORB object
            cv::Ptr<cv::ORB> orb = cv::ORB::create();

            // Detect ORB features in the two images
            orb->detectAndCompute(greyMat1, cv::Mat(), keypoints1, descriptors1);
            orb->detectAndCompute(greyMat2, cv::Mat(), keypoints2, descriptors2);
        }

        // Match the two sets of descriptors

        matcher->match(descriptors1, descriptors2, matches);

        return 1;
    }

    orb(cv::Mat &_Img1, cv::Mat &_Img2)
    {
        img1 = &_Img1;
        img2 = &_Img2;

        matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        // Check if Cuda avialabe
        if (cv::cuda::getCudaEnabledDeviceCount())
        {
            isGPU = true;
        }
        else
        {
            isGPU = false;
        }
    }
};
