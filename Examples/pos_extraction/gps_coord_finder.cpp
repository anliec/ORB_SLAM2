#include "gps_coord_finder.h"
#include "kabsch.h"
#include <opencv2/core/eigen.hpp>

cv::Mat findTransformToGlobalCoord(const std::vector<CoordGPS> &worldPoses, const std::vector<cv::Mat> &slamPoses)
{
    assert(worldPoses.size() == slamPoses.size());
    Eigen::Matrix3Xd worldPoints, slamPoints;

    std::vector<CoordGPS>::const_iterator worldPoint = worldPoses.begin();
    std::vector<cv::Mat>::const_iterator slamPoint = slamPoses.begin();
    for(unsigned i=0; worldPoint!=worldPoses.end() && slamPoint!=slamPoses.end() ; worldPoint++, slamPoint++, i++){
        if(slamPoint->empty()){
            continue;
        }
        worldPoints(0,i) = worldPoint->lat;
        worldPoints(1,i) = worldPoint->lon;
        worldPoints(2,i) = worldPoint->alt;
        slamPoints(0,i) = slamPoint->at<double>(0,3);
        slamPoints(1,i) = slamPoint->at<double>(1,3);
        slamPoints(2,i) = slamPoint->at<double>(2,3);
    }

    Eigen::Affine3d transformation = Find3DAffineTransform(slamPoints, worldPoints);

    cv::Mat ret;
    cv::eigen2cv(transformation.matrix(), ret);

    return ret;
}

std::vector<CoordGPS> getGlobalCoord(const std::vector<SignCoordinate> &slamPosition, const cv::Mat &transform)
{
    std::vector<CoordGPS> worldCoords;

    cv::Mat translation = transform(cv::Rect2i(0,3,3,1)).clone();
    cv::Mat rotation = transform(cv::Rect2i(0,0,3,3)).clone();

    for(const SignCoordinate &slamCoord : slamPosition){
        cv::Mat worldPos = rotation * cv::Mat(slamCoord.p) + translation;
        worldCoords.emplace_back(worldPos, slamCoord.signId);
    }

    return worldCoords;
}
