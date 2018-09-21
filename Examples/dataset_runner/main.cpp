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
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include <glob.h> // glob(), globfree()
#include <string.h> // memset()

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

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

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_dataset" << endl;
        return 1;
    }
    std::string datasetPath = argv[2];
    std::string configFilePath(datasetPath + "config.yaml");
    std::string imagesGlob(datasetPath + "images/*.jpg");

    std::cout << imagesGlob << std::endl;

    std::vector<std::string> imagesPaths = glob(imagesGlob);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],configFilePath,ORB_SLAM2::System::MONOCULAR,true);

    cv::FileStorage fSettings(configFilePath, cv::FileStorage::READ);

    double fps = fSettings["Camera.fps"];
    double spf = 1 / fps;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << imagesPaths.size() << endl << endl;

    // Main loop
    cv::Mat im;
    double tframe = 0.0;
    for(std::string &imagePath : imagesPaths)
    {
        // Read image from file
        im = cv::imread(imagePath,cv::IMREAD_UNCHANGED);
        tframe += spf;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << tframe << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        if(ttrack<spf)
            std::this_thread::sleep_for(std::chrono::microseconds(int((spf-ttrack)*1e6)));
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

