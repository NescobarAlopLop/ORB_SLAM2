#include <getopt.h>
#include<iostream>
#include<algorithm>
#include<fstream>

#include<chrono>

#include<opencv2/core/core.hpp>
#include<System.h>
using namespace std;


void LoadImages(const string &strImagePath, const string &strPathTimes,
	vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
	if(argc != 4)
	{
		cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings vide_device_id" << endl;
		cerr << endl << "Example:\n./mono_live_from_usb_cam Vocabulary/ORBvoc.txt Examples/Monocular/TUM_512.yaml 0 1" << endl;
		return 1;
	}
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	// Vector for tracking time statistics
	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	// Main loop
	cv::Mat im;
	int deviceID = stoi(argv[3]);
	int pnp_version = stoi(argv[4]);
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API
	cv::VideoCapture cap;
	cap.open(deviceID, apiID);
	// check if we succeeded
	if (!cap.isOpened()) {
		cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	for(int ni=0; ni<10000; ni++)
	{
		// Read image from camera
		cap.read(im);

		if(im.empty())
		{
			cerr << endl << "Blank frame grabbed" << endl;
//			return 1;
			continue;
		}

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#elif COMPILEDWITHC17
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

		// Pass the image to the SLAM system
		SLAM.TrackMonocular(im, ni, pnp_version);

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#elif COMPILEDWITHC17
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		string file_name = "/home/george/Pictures/test_sequence_laptop/4/starry_night-" + to_string(t1.time_since_epoch().count()) + ".png";
		imwrite(file_name, im);
	}

	// Stop all threads
	SLAM.Shutdown();

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return EXIT_SUCCESS;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
	vector<string> &vstrImages, vector<double> &vTimeStamps)
{
	ifstream fTimes;
	fTimes.open(strPathTimes.c_str());
	vTimeStamps.reserve(5000);
	vstrImages.reserve(5000);
	while(!fTimes.eof())
	{
		string s;
		getline(fTimes,s);
		if(!s.empty())
		{
			stringstream ss;
			ss << s;
			vstrImages.push_back(ss.str() + ".png");
			double t;
			ss >> t;
			vTimeStamps.push_back(t/1e9);
		}
	}
}
