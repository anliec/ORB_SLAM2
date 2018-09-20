#include "gps_coord_finder.h"
#include "kabsch.h"
#include <opencv2/core/eigen.hpp>

cv::Mat findTransformToGlobalCoord(const std::vector<CoordGPS> &worldPoses, const std::vector<cv::Mat> &slamPoses)
{
    assert(worldPoses.size() == slamPoses.size());

    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> pointsPaires;

    std::cout << std::endl << slamPoses.back().type() << std::endl;

    std::vector<CoordGPS>::const_iterator worldPoint = worldPoses.begin();
    std::vector<cv::Mat>::const_iterator slamPoint = slamPoses.begin();
    for(unsigned i=0; worldPoint!=worldPoses.end() && slamPoint!=slamPoses.end() ; worldPoint++, slamPoint++, i++){
        if(slamPoint->empty()){
            i--;
            continue;
        }
        Eigen::Vector3d wp, sp;
        wp[0] = worldPoint->lat;
        wp[1] = worldPoint->lon;
        wp[2] = worldPoint->alt;
        sp[0] = slamPoint->at<float>(0,3);
        sp[1] = slamPoint->at<float>(1,3);
        sp[2] = slamPoint->at<float>(2,3);
        std::pair<Eigen::Vector3d,Eigen::Vector3d> paire;
        paire.first = wp;
        paire.second = sp;
        pointsPaires.push_back(paire);
    }
    Eigen::Matrix<double, 3, Eigen::Dynamic> worldPoints(3,pointsPaires.size()), slamPoints(3,pointsPaires.size());
    int i = 0;
    for(const std::pair<Eigen::Vector3d,Eigen::Vector3d> &p : pointsPaires){
        worldPoints.col(i) = p.first;
        slamPoints.col(i) = p.second;
        i++;
    }


    std::cout << slamPoints << std::endl;
    std::cout << worldPoints << std::endl;

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> transformation = Eigen::umeyama(slamPoints, worldPoints, true);

    for(int i=0 ; i<slamPoints.cols() ; ++i){
        std::cout << transformation * slamPoints.col(i).homogeneous() - worldPoints.col(i).homogeneous() << std::endl;
    }

    cv::Mat ret;
    cv::eigen2cv(transformation, ret);

//    std::cout << std::endl << transformation << std::endl;
//    std::cout << ret << std::endl << std::endl;

    cv::Mat translation = ret(cv::Rect2i(3,0,1,3)).clone();
    translation.convertTo(translation, CV_64F);
    cv::Mat rotation = ret(cv::Rect2i(0,0,3,3)).clone();
    rotation.convertTo(rotation, CV_64F);

//    std::cout << translation << std::endl;
//    std::cout << rotation << std::endl;
    i = 0;
    for(const cv::Mat &slamCoord : slamPoses){
        if(slamCoord.empty()) continue;
        cv::Mat p = slamCoord(cv::Rect2i(3,0,1,3)).clone();
        p.convertTo(p, CV_64F);
        cv::Vec3d wp;
        wp[0] = worldPoses[i].lat;
        wp[1] = worldPoses[i].lon;
        wp[2] = worldPoses[i].alt;
        std::cout << (rotation * p) + translation - cv::Mat(wp) << std::endl;
        i++;
    }

    return ret;
}

std::vector<CoordGPS> getGlobalCoord(const std::vector<SignCoordinate> &slamPosition, const cv::Mat &transform)
{
    std::vector<CoordGPS> worldCoords;

    cv::Mat translation = transform(cv::Rect2i(3,0,1,3)).clone();
    translation.convertTo(translation, CV_64F);
    cv::Mat rotation = transform(cv::Rect2i(0,0,3,3)).clone();
    rotation.convertTo(rotation, CV_64F);

    for(const SignCoordinate &slamCoord : slamPosition){
        cv::Mat p = cv::Mat(slamCoord.p);
        p.convertTo(p, CV_64F);
        cv::Mat worldPos = rotation * p + translation;
        worldCoords.emplace_back(worldPos, slamCoord.signId);
    }

    return worldCoords;
}





























