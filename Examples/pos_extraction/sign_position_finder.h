#ifndef SIGN_POSITION_FINDER_H
#define SIGN_POSITION_FINDER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include "pose_finder.h"
#include "bounding_boxes_manager.h"

struct SignCoordinate
{
    cv::Vec3d p;
    int signId = -1;
    int rayUsed = -1;
    SignCoordinate(const int &id, const double &posX, const double &posY, const double &posZ, const int &rayCount=-1):
        signId(id), p(posX, posY, posZ), rayUsed(rayCount)
    {

    }
    SignCoordinate(const int &id, const cv::Vec3d &pos, const int &rayCount=-1):
        signId(id), p(pos), rayUsed(rayCount)
    {

    }
};

std::vector<SignCoordinate> findSignCoordinates(const std::vector<SignPointer> &rays, const std::vector<cv::Mat> &poses);

#endif // SIGN_POSITION_FINDER_H
