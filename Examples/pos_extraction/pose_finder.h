#ifndef POSE_FINDER_H
#define POSE_FINDER_H

#include <vector>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

std::vector<cv::Mat> extract_local_pose_from_images(const std::string& pathToVoc, const std::string& pathToSettings, const std::string& pathToVideo);

std::vector<cv::Mat> convertMovementToPose(const std::vector<cv::Mat>& moves);

#endif // POSE_FINDER_H
