#include <iostream>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

int main(int argc, char** argv)
{

	vector<int> camera_indexes;

	if (argc == 2)
	{
		int deviceId = stoi(argv[1]);
		cout << "testing specific camera id: " << deviceId << endl;
		camera_indexes.push_back(deviceId);
	}
	else
	{
		for (int i = 0; i < 7; ++i)
		{
			camera_indexes.push_back(i);
		}
	}

	cv::Mat im;
	cv::VideoCapture cap;
	string window_name = "camera tester";
	cv::namedWindow(window_name, cv::WINDOW_KEEPRATIO);

	for (auto id : camera_indexes)
	{
		cap.open(id);
		if (!cap.isOpened())
		{
			cerr << "ERROR! Unable to open camera id: " << id << endl;
			continue;
		}

		string infoString;
		infoString = "/dev/video" + to_string(id);
		for (int ni = 0; ni < 50; ni++)
		{

			cout << "showing camera: " << infoString << endl;
			cap.read(im);
			if (im.empty())
			{
				cerr << "Failed to read a frame, skipping to next one" << id << endl;
				continue;
			}
			cv::putText(im,
				infoString,
				cv::Point(40, 50),
				cv::FONT_HERSHEY_SIMPLEX,
				1.8,
				cv::Scalar(255, 255, 255),
				2);
			cv::imshow(window_name, im);
			cv::waitKey(30);
		}
	}
	cv::destroyAllWindows();

	return 0;
}
