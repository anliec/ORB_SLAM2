#include "data_reader.h"

#include <fstream>

std::vector<CoordGPS> readCoordinates(const std::__cxx11::string &coordinateFilePath, const bool& skipHeader)
{
    std::ifstream csvFile(coordinateFilePath);
    std::vector<CoordGPS> coords;
    std::string line;

    if(skipHeader){
        std::getline(csvFile, line);
    }

    while(std::getline(csvFile, line)){
        std::stringstream lineStream(line);
        std::string cell;

        std::getline(lineStream, cell, ',');
        int frame_number = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        int frame_id = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        double lat = atof(cell.c_str());
        std::getline(lineStream, cell, ',');
        double lon = atof(cell.c_str());
        std::getline(lineStream, cell, ',');
        double alt = atof(cell.c_str());

        coords.emplace_back(frame_number, frame_id, lat, lon, alt);
    }

    return coords;
}

std::vector<BoundingBox> readBoundingBoxes(const std::string &BoundingBoxFilePath, const bool &skipHeader)
{
    std::ifstream csvFile(BoundingBoxFilePath);
    std::vector<BoundingBox> boundingBoxes;
    std::string line;

    if(skipHeader){
        std::getline(csvFile, line);
    }

    while(std::getline(csvFile, line)){
        std::stringstream lineStream(line);
        std::string cell;

        std::getline(lineStream, cell, ',');
        int frame_number = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        int id = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        int x = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        int y = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        int w = atoi(cell.c_str());
        std::getline(lineStream, cell, ',');
        int h = atoi(cell.c_str());

        boundingBoxes.emplace_back(frame_number, id, x, y, w, h);
    }

    return boundingBoxes;
}

std::pair<cv::Mat, cv::Mat> readCalibrationMatrix(const std::string &configFilePath)
{
    cv::FileStorage fSettings(configFilePath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    std::pair<cv::Mat, cv::Mat> ret;
    ret.first = K;
    ret.second = DistCoef;
    return ret;
}






















