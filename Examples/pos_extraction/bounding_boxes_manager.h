#ifndef BOUNDING_BOXES_MANAGER_H
#define BOUNDING_BOXES_MANAGER_H

#include "data_reader.h"
#include <opencv2/core/core.hpp>

#include <iostream>

struct UndistortedBoundingBox{
    int frameNumber;
    int frameName;
    double x;
    double y;
    double w;
    double h;
    int signId;


    UndistortedBoundingBox(const int& frame=0, const int& id=0, const double& x=0.0, const double& y=0.0, const double& w=0.0, const double& h=0.0, const int& signId=-1):
        frameNumber(frame), frameName(id), x(x), y(y), w(w), h(h), signId(signId)
    {

    }
};

std::vector<UndistortedBoundingBox> undistordBoundingBoxes(const std::vector<BoundingBox> &bb, const std::pair<cv::Mat,cv::Mat> &calibration);

struct SignPointer{
    int frameNumber;
    int frameName;
    int signId;
    cv::Vec<float,4> v;

    SignPointer():
        frameNumber(-1), frameName(-1), signId(-1)
    {
        v = cv::Vec<float,4>(-1.0,-1.0,-1.0,-1.0);
    }

    SignPointer(const int& frame, const int& id, const double& x, const double& y, const int &signId):
        frameNumber(frame), frameName(id), signId(signId)
    {
        v = cv::Vec<float,4>(x,y,1.0,1.0);
    }

    SignPointer(const UndistortedBoundingBox& bb):
        frameNumber(bb.frameNumber), frameName(bb.frameName), signId(bb.signId)
    {
        double x = bb.x + (bb.w / 2.0);
        double y = bb.y + (bb.h / 2.0);
        // fx is 1.0 after undistord: https://docs.opencv.org/master/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e
        v = cv::Vec<float,4>(x,y,1.0,1.0);
    }
};

std::vector<SignPointer> getSignPointerFromBoundingBoxes(const std::vector<UndistortedBoundingBox>& bb);

std::vector<SignPointer> movePointerToWorldCoordinate(const std::vector<SignPointer>& pointers, const std::vector<cv::Mat>& cam_poses);

#endif // BOUNDING_BOXES_MANAGER_H
