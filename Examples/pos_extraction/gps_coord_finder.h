#ifndef GPS_COORD_FINDER_H
#define GPS_COORD_FINDER_H

#include <opencv2/core/core.hpp>
#include <vector>

#include "data_reader.h"
#include "sign_position_finder.h"

cv::Mat findTransformToGlobalCoord(const std::vector<CoordGPS> &worldPoses, const std::vector<cv::Mat> &slamPoses);

std::vector<SignCoordinate> getGlobalCoord(const std::vector<SignCoordinate> &slamPosition, const cv::Mat &transform);

#endif // GPS_COORD_FINDER_H
