//
// Created by ge on 14/11/19.
//
/**
 *
 * This file is a modification of ORB-SLAM2.
 *
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

#include<opencv2/core/core.hpp>
#include <unistd.h>
#include<System.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>


using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tello path_to_vocabulary path_to_settings"
                        " (/dev/videox -> x, rdp stream -> full url)" << endl;
        cout << "Example command: ./mono_tello ../../Vocabulary/ORBvoc.txt tello.yaml" << endl;
        return 1;
    }

    auto rows = 720;
    auto cols = 960;
    auto colors = 3;
    auto fps = 25;
    auto sizeOfData = rows * cols * colors;            // TODO: move magic numbers to config file

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    const char *fifo_name = "/tmp/images.fifo";
    std::ifstream fifo;
    char buf[sizeOfData];
    cv::namedWindow( "read from pipe", CV_WINDOW_AUTOSIZE );

    cout << "Start processing video from named pipe ..." << endl;
    // Main loop
    for(; ; )
    {
        fifo.open(fifo_name, std::ios_base::in);

        // Read image from pipe
        fifo.read(buf, sizeOfData);
        if (!fifo.good()) {
            std::cerr << "Read failed" << std::endl;
            fifo.close();
            continue;
        }

        // Read image from buf to cvmat
        cv::Mat imageWithData2 = cv::Mat(rows, cols, CV_8UC3, buf);//.clone();
        fifo.close();

        cv::imshow("read from pipe", imageWithData2);
        if(cv::waitKey(1) == 27) // Wait for 'esc' key press to exit
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/fps));

        double tframe = 0.2;        // TODO: magic number fix according to original mono

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(imageWithData2,tframe);

    }

    // Stop all threads
    SLAM.Shutdown();

    cvDestroyAllWindows();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
