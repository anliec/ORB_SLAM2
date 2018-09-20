#include "pose_finder.h"

#include "../../include/System.h"
#include <iostream>
#include <glob.h> // glob(), globfree()
#include <string.h> // memset()
#include <vector>
#include <stdexcept>
#include <string>
#include <sstream>

std::vector<std::string> glob(const std::string& pattern) {
    using namespace std;

    // glob struct resides on the stack
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));

    // do the glob operation
    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        globfree(&glob_result);
        stringstream ss;
        ss << "glob() failed with return_value " << return_value << endl;
        throw std::runtime_error(ss.str());
    }

    // collect all the filenames into a std::list<std::string>
    vector<string> filenames;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
        filenames.push_back(string(glob_result.gl_pathv[i]));
    }

    // cleanup
    globfree(&glob_result);

    // done
    return filenames;
}

std::vector<cv::Mat> extract_local_pose_from_images(const std::__cxx11::string &pathToVoc, const std::__cxx11::string &pathToSettings,
                                                    const std::__cxx11::string &pathToVideo)
{
    cv::FileStorage fSettings(pathToSettings, cv::FileStorage::READ);
    double fps = fSettings["Camera.fps"];

    std::cout << pathToVideo << std::endl;
    std::vector<std::string> framesPaths = glob(pathToVideo);
    if(framesPaths.size() == 0){
        cerr << "Cannot open frames: " << pathToVideo << endl;
        exit(1);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System slam(pathToVoc,pathToSettings,ORB_SLAM2::System::MONOCULAR,false);

    std::vector<cv::Mat> poses;
    poses.reserve(framesPaths.size());
    cv::Mat frame;
    int frame_number = 0;
    for(std::string framePath : framesPaths){
        frame = cv::imread(framePath);
        if(frame.empty()) //if not success, break loop
        {
            cerr << "Cannot read frame " << framePath << endl;
            exit(1);
        }

        poses.push_back(slam.TrackMonocular(frame, double(frame_number) / fps));
//        std::cout << frame_number << ":" << std::endl << poses[frame_number]  << std::endl;
        frame_number++;
    }

    // Stop all threads
    slam.Shutdown();

    return poses;
}

/// cf: https://github.com/raulmur/ORB_SLAM2/pull/102
std::vector<cv::Mat> convertMovementToPose(const std::vector<cv::Mat> &moves)
{
    /* global left handed coordinate system */
    cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                       -1, 1,-1, 1,
                                                       -1,-1, 1, 1,
                                                        1, 1, 1, 1);
    const cv::Mat flipSign2 = (cv::Mat_<float>(4,4) <<  -1, 1, 1, 1,
                                                        -1, 1, 1, 1,
                                                         1,-1,-1,-1,
                                                         1, 1, 1, 1);

    std::vector<cv::Mat> poses;
    poses.reserve(moves.size());

    for(const cv::Mat& pose : moves){
        if(pose.empty()){
            poses.push_back(pose.clone());
            continue;
        }
        //prev_pose * T = pose
        cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
        world_lh = world_lh * translation;
        pose_prev = pose;
        cv::Mat transform = world_lh.clone();
        transform.mul(flipSign2);
        poses.push_back(transform);
    }

    return poses;
}


















