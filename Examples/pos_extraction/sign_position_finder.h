#ifndef SIGN_POSITION_FINDER_H
#define SIGN_POSITION_FINDER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include "pose_finder.h"
#include "bounding_boxes_manager.h"

struct SignCoordinate
{
    cv::Vec3d p;
    int signId;
    SignCoordinate(const int &id, const double &posX, const double &posY, const double &posZ):
        signId(id), p(posX, posY, posZ)
    {

    }
    SignCoordinate(const int &id, const cv::Vec3d &pos):
        signId(id), p(pos)
    {

    }
};

std::vector<SignCoordinate> findSignCoordinates(const std::vector<SignPointer> &rays, const std::vector<cv::Mat> &poses);

#endif // SIGN_POSITION_FINDER_H
