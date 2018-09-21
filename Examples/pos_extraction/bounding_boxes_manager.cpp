#include "bounding_boxes_manager.h"
#include <opencv2/opencv.hpp>

std::vector<UndistortedBoundingBox> undistordBoundingBoxes(const std::vector<BoundingBox> &bb, const std::pair<cv::Mat, cv::Mat> &calibration)
{
    std::vector<cv::Point2f> pointsToUndistord, undistordedPoints;
    pointsToUndistord.reserve(bb.size() * 2);
    undistordedPoints.reserve(pointsToUndistord.size());

    for(BoundingBox b : bb){
        pointsToUndistord.emplace_back(b.x, b.y);
        pointsToUndistord.emplace_back(b.x + b.w, b.y + b.h);
    }

    cv::undistortPoints(pointsToUndistord, undistordedPoints, calibration.first, calibration.second);

    std::vector<UndistortedBoundingBox> undistortedBoundingBoxes;
    undistortedBoundingBoxes.reserve(bb.size());

    auto p=undistordedPoints.begin(); auto b=bb.begin();
    for( ; p!=undistordedPoints.end() && b!=bb.end() ; p++, b++){
        cv::Point2f& ul = *p;
        cv::Point2f& dr = *(++p);
        undistortedBoundingBoxes.emplace_back(b->frameNumber, b->frameName, ul.x, ul.y, dr.x - ul.x, dr.y - ul.y, b->signId);
    }

    return undistortedBoundingBoxes;
}

std::vector<SignPointer> getSignPointerFromBoundingBoxes(const std::vector<UndistortedBoundingBox> &bb)
{
    std::vector<SignPointer> pointers;
    pointers.reserve(bb.size());
    for(UndistortedBoundingBox b : bb){
        pointers.emplace_back(b);
    }
    return pointers;
}

std::vector<SignPointer> movePointerToWorldCoordinate(const std::vector<SignPointer> &pointers, const std::vector<cv::Mat> &cam_poses)
{
    std::vector<SignPointer> ret;
    ret.reserve(pointers.size());

    for(const SignPointer &p : pointers){
        const cv::Mat& cam = cam_poses[p.frameNumber];
        if(cam.empty())
            continue;
        const cv::Mat& reverseMove = cam;//.inv();
        SignPointer np = p;
        cv::Mat v = reverseMove * cv::Mat(p.v, false);
        v.copyTo(cv::Mat(np.v, false));
        ret.push_back(np);
    }

    return ret;
}








