/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include <cmath>

#include<opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"

#include"../../include/System.h"

#include "pose_finder.h"
#include "data_reader.h"
#include "bounding_boxes_manager.h"
#include "sign_position_finder.h"
#include "gps_coord_finder.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: " << argv[0] << " path_to_vocabulary path_to_run_directory" << endl;
        return 1;
    }    
    std::string argv2 = argv[2];
    std::string configPath = argv2 + "config.yaml";
    std::string framesPath = argv2 + "images/*.jpg";
    std::string bbPath = argv2 + "bb.csv";
    std::string coordPath = argv2 + "coord.csv";

    //==================================================//
    // Use OrbSlam2 to get camera position              //
    //==================================================//

    std::vector<cv::Mat> poses = extract_local_pose_from_images(argv[1], configPath, framesPath);
    poses = convertMovementToPose(poses);

    //==================================================//
    // Load useful data                                 //
    //==================================================//

    std::vector<CoordGPS> coords = readCoordinates(coordPath);
    std::vector<BoundingBox> boundingBoxes = readBoundingBoxes(bbPath);
    std::pair<cv::Mat,cv::Mat> calibration = readCalibrationMatrix(configPath);

    //==================================================//
    // Undistord bounding boxes and get there direction //
    //==================================================//

    std::vector<UndistortedBoundingBox> correctedBoundingBoxes = undistordBoundingBoxes(boundingBoxes, calibration);
    std::vector<SignPointer> signPointers = getSignPointerFromBoundingBoxes(correctedBoundingBoxes);
    signPointers = movePointerToWorldCoordinate(signPointers, poses);

    //==================================================//
    // Find sign position acording to rays              //
    //==================================================//

    std::vector<SignCoordinate> signPos = findSignCoordinates(signPointers, poses);

    //==================================================//
    // Transform position the world coordiantes         //
    //==================================================//

    cv::Mat transform = findTransformToGlobalCoord(coords, poses);
    std::vector<CoordGPS> signWorldPos = getGlobalCoord(signPos, transform);

    //==================================================//
    // Write results and debug infos                    //
    //==================================================//

    std::ofstream out(argv2 + "cam.txt");
    out << std::fixed << std::setprecision(8);
    out << "X,Y,Z,rX,rY,rZ,id" << std::endl;
    int i = -1;
    for(cv::Mat& pose : poses){
        i++;
        if(pose.empty())
            continue;
        double sy = std::sqrt(pose.at<double>(2,1) * pose.at<double>(2,1) +  pose.at<double>(2,2) * pose.at<double>(2,2) );
        double thetaX, thetaY, thetaZ;
        if(sy > 1e-6){ // if rotation matrix is not singular
            thetaX = std::atan2(pose.at<double>(2,1), pose.at<double>(2,2));
            thetaY = std::atan2(-pose.at<double>(2,0), sy);
            thetaZ = std::atan2(pose.at<double>(1,0), pose.at<double>(0,0));
        }
        else{
            thetaX = std::atan2(-pose.at<double>(1,2), pose.at<double>(1,1));
            thetaY = std::atan2(-pose.at<double>(2,0), sy);
            thetaZ = 0;
        }
        out << pose.at<double>(0, 3) << "," << pose.at<double>(1, 3) << "," << pose.at<double>(2, 3) << "," << thetaX << "," << thetaY << "," << thetaZ << "," << i << std::endl;
    }

    out.close();

    out.open(argv2 + "ray.txt");
    out << "X,Y,Z,cX,cY,cZ,id" << std::endl;
    i = -1;
    for(SignPointer& p : signPointers){
        i++;
        cv::Mat& cam = poses[p.frameNumber];
        out << p.v[0] << "," << p.v[1] << "," << p.v[2] << "," << cam.at<float>(0, 3) << "," << cam.at<float>(1, 3) << "," << cam.at<float>(2, 3) << "," << i << std::endl;
    }

    out.close();

    out.open(argv2 + "pos.txt");
    out << "X,Y,Z,id,rayUsed" << std::endl;
    for(SignCoordinate& c : signPos){
        out << c.p[0] << "," << c.p[1] << "," << c.p[2] << "," << c.signId << ',' << c.rayUsed << std::endl;
    }

    out.close();

    out.open(argv2 + "worldPos.txt");
    out << "lat,lon,alt,id" << std::endl;
    for(CoordGPS c : signWorldPos){
        out << c.lat << "," << c.lon << "," << c.alt << "," << c.frameId << std::endl;
    }

    out.close();

    out.open(argv2 + "worldCamPos.txt");
    out << "lat,lon,alt,id" << std::endl;
    std::vector<SignCoordinate> coord;
    for(const cv::Mat &m : poses){
        if(m.empty()) continue;
        coord.emplace_back(coord.size(), m(cv::Rect2i(3,0,1,3)));
    }
    std::vector<CoordGPS> camWorldPos = getGlobalCoord(coord, transform);
    for(CoordGPS c : camWorldPos){
        out << c.lat << "," << c.lon << "," << c.alt << "," << c.frameId << std::endl;
    }

    out.close();

    return 0;
}



