#ifndef DATA_READER_H
#define DATA_READER_H

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>

struct CoordGPS{
    int frameNumber;
    int frameId;
    double lat;
    double lon;
    double alt;

    CoordGPS(const int& frame=0, const int& id=0, const double& gpsLat=0.0, const double& gpsLon=0.0, const double& gpsAlt=0.0):
        frameNumber(frame), frameId(id), lat(gpsLat), lon(gpsLon), alt(gpsAlt)
    {

    }
    CoordGPS(const cv::Vec3d &coord,  const int& id=0, const int& frame=0 ):
        frameNumber(frame), frameId(id), lat(coord(0)), lon(coord(1)), alt(coord(2))
    {

    }
};

std::vector<CoordGPS> readCoordinates(const std::string& coordinateFilePath, const bool &skipHeader=true);

struct BoundingBox{
    int frameNumber;
    int frameName;
    int x;
    int y;
    int w;
    int h;
    int signId;

    BoundingBox(const int& frame, const int& id, const int& x, const int& y, const int& w, const int& h, const int& signId):
        frameNumber(frame), frameName(id), x(x), y(y), w(w), h(h), signId(signId)
    {

    }
};

std::vector<BoundingBox> readBoundingBoxes(const std::string& BoundingBoxFilePath, const bool &skipHeader=true);

std::pair<cv::Mat, cv::Mat> readCalibrationMatrix(const std::string& configFilePath);

#endif // DATA_READER_H
