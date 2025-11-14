/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>
#include <string> // For std::string

#include<opencv2/core/core.hpp>
#include<opencv2/videoio.hpp> // For cv::VideoCapture
#include<opencv2/highgui/highgui.hpp> // For cv::imshow

#include<System.h>
#include "ImuTypes.h"

using namespace std;

// Function to load IMU data from a file
void LoadIMUFromFile(const string &strImuPath, vector<ORB_SLAM3::IMU::Point> &vImuMeas)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    if (!fImu.is_open()) {
        cerr << "ERROR: Could not open IMU file: " << strImuPath << endl;
        return;
    }

    string s;
    while(getline(fImu,s))
    {
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            stringstream ss(s);
            string item;
            double data[7];
            int count = 0;
            while (getline(ss, item, ' ')) { // Assuming space-separated values
                data[count++] = stod(item);
            }

            // Assuming format: timestamp ax ay az gx gy gz
            // Adjust indices if your imu_euroc.txt format is different
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(data[1], data[2], data[3], // Accel
                                                     data[4], data[5], data[6], // Gyro
                                                     data[0]/1e9)); // Timestamp in seconds
        }
    }
    fImu.close();
}


double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings [imu_data_file]" << endl;
        return 1;
    }

    // Initialize camera
    cv::VideoCapture cap(0); // Open the default camera (index 0)
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }

    // Load IMU data from file if provided
    vector<ORB_SLAM3::IMU::Point> allImuMeasurements;
    if (argc >= 4) {
        string imu_data_file = argv[3];
        LoadIMUFromFile(imu_data_file, allImuMeasurements);
        if (allImuMeasurements.empty()) {
            cerr << "WARNING: No IMU data loaded from file: " << imu_data_file << endl;
        } else {
            cout << "Loaded " << allImuMeasurements.size() << " IMU measurements from " << imu_data_file << endl;
        }
    }

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    int currentImuIdx = 0;

    // Main loop for live data
    while (true)
    {
        cap >> im; // Capture a new frame
        if (im.empty()) {
            cerr << endl << "Failed to capture image" << endl;
            break;
        }

        // Get current timestamp for the camera frame
        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(
                            std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Collect IMU measurements up to the current camera frame timestamp
        vImuMeas.clear();
        while (currentImuIdx < allImuMeasurements.size() && allImuMeasurements[currentImuIdx].t <= tframe) {
            vImuMeas.push_back(allImuMeasurements[currentImuIdx]);
            currentImuIdx++;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vImuMeas);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        ttrack_tot += ttrack;

        // Display the frame (optional)
        cv::imshow("ORB-SLAM3 Live", im);
        if (cv::waitKey(1) == 27) // 27 is the ASCII code for ESC
            break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}
