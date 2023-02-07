#pragma once

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <bits/stdc++.h>

struct EGO_MOTION
{
    static std::vector<cv::KeyPoint> evaluate_groundtruths(std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, double delta_time, float focal_length)
    {

        std::vector<cv::KeyPoint> ground_truth_points;

        double pixel_size = 0.00000112; // for the 	Sony IMX219

        std::vector<cv::KeyPoint> groundTruths;
        for (int i = 0; i < matches.size(); i++) // Note: make this a parrell process
        {
            if (matches[i].distance < 100)
            {
                // Find key matching key point indexes
                int ind1 = matches[i].trainIdx;
                int ind2 = matches[i].queryIdx;

                // Get matching coords from both images
                cv::Point coord1 = keypoints1[ind1].pt;
                cv::Point coord2 = keypoints2[ind2].pt;

                // Calculate Depth;
                double depth = (delta_time * focal_length) / (sqrt(pow(coord2.x - coord1.x, 2) + pow(coord2.y - coord1.y, 2)) * pixel_size);

               

                if (!isinf(depth))
                {

 		    // std::cout << depth << std::endl;
                    cv::KeyPoint kp;
                    kp.pt = coord2;
                    kp.response = depth;

                    ground_truth_points.push_back(kp);
                }
            }
        }

        return ground_truth_points;
    }
};
