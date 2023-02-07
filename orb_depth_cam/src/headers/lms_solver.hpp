#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
using namespace cv;
struct LMS_OPTIMSER
{
    static double optimizeScale(const cv::Mat &relative_depth_image, const std::vector<cv::KeyPoint> &ground_truth_points)
    {
        int n = ground_truth_points.size();
        cv::Mat A(n, 1, CV_64F);  //create [n,1] matrix
        cv::Mat B(n, 1, CV_64F);


        for (int i = 0; i < n; i++)
        {
            int x = ground_truth_points[i].pt.x;
            int y = ground_truth_points[i].pt.y;
            A.at<double>(i, 0) = (int)relative_depth_image.at<uchar>(y, x);
            B.at<double>(i, 0) = ground_truth_points[i].response;

           //std::cout << "GT: " <<  B.at<double>(i, 0) << " REL: " << A.at<double>(i, 0) << std::endl;
        }

        // Ax = B. given an A and B matrix. solve for x using LMS
        cv::Mat x;
        cv::solve(A, B, x, cv::DECOMP_SVD);

        return x.at<double>(0, 0);
    }
};
