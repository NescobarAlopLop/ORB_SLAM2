#include <getopt.h>
#include<iostream>
#include<algorithm>
#include<fstream>

#include<chrono>

#include<opencv2/core/core.hpp>
#include <unistd.h>
#include<System.h>
#include <string_view>
#include <string>
using namespace std;


void LoadImagesEuroc(
	const string &cam0Path,
	vector<string> &vstrImages,
	vector<double> &vTimeStamps,
	int &framesToSkip
);

void parse_arguments(int argc, char* const* argv);
static int mode;
static int mode_int;
static int camera_idx;
static int camera_idx_int;
static int camera_calib;
static int camera_calib_int;
static int vocabulary_path_int;
static int dataset_int;
static int kf_trajectory;
static int skip_frames = 0;
static int new_pnp = 0;

string vocabularyPath;
string datasetPath;
string cameraSettingsPath;

enum OperationModes
{
	LIVE_CAM = 0,
	DATASET = 1,
};

/* Flag set by ‘--verbose’. */
static int verbose_flag;

int main(int argc, char **argv)
{
	parse_arguments(argc, argv);

	/* Instead of reporting ‘--verbose’
	   and ‘--brief’ as they are encountered,
	   we report the final status resulting from them. */
	if (verbose_flag)
		puts ("verbose flag is set");

	/* Print any remaining command line arguments (not options). */
	if (optind < argc)
	{
		printf ("non-option ARGV-elements: ");
		while (optind < argc)
			printf ("%s ", argv[optind++]);
		putchar ('\n');
	}


	if (optind < argc) {
		printf("non-option ARGV-elements: ");
		while (optind < argc)
			printf("%s ", argv[optind++]);
		printf("\n");
	}
	cout << "mode " << mode << endl;
	cout << "mode_int " << mode_int << endl;
	cout << "camera_idx " << camera_idx << endl;
	cout << "camera_idx_int " << camera_idx_int << endl;
	cout << "camera_calib " << camera_calib << endl;
	cout << "camera_calib_int " <<camera_calib_int << endl;
	cout << "cameraSettingsPath " << cameraSettingsPath << endl;
	cout << "vocabulary_path " << vocabularyPath << endl;
	cout << "vocabulary_path_int " << vocabulary_path_int << endl;
	cout << "dataset " << datasetPath << endl;
	cout << "dataset_int " << dataset_int << endl;
	cout << "kf_trajectory " << kf_trajectory << endl;
	cout << "skip_frames " << skip_frames << endl;
	cout << "new_pnp " << new_pnp << endl;
	cout << "verbose_flag " << verbose_flag << endl;


	cv::Mat im;
	cv::VideoCapture cap;
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API


	int totalImages = 10000;
	switch(mode)
	{
	case OperationModes::LIVE_CAM:
		cap.open(camera_idx, apiID);

		if (!cap.isOpened()) {
			cerr << "ERROR! Unable to open camera\n";
			return -1;
		}

		break;
	case OperationModes::DATASET:
		LoadImagesEuroc(datasetPath, vstrImageFilenames, vTimestamps, skip_frames);
		totalImages = vstrImageFilenames.size();
		break;
	default:
		cout << "No such operation mode" << endl;
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(vocabularyPath, cameraSettingsPath,ORB_SLAM2::System::MONOCULAR,true);

	// Vector for tracking time statistics
	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	double tframe;
	for(int ni = 0; ni < totalImages; ++ni)
	{
		// Read image from camera
		if (mode == OperationModes::DATASET)
		{
			cout << "loading image " << vstrImageFilenames[ni] << endl;
//			im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
			im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_GRAYSCALE);
			tframe = vTimestamps[ni];
		}
		else
		{
			cap.read(im);
		}

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
		SLAM.TrackMonocular(im, ni, new_pnp);

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#elif COMPILEDWITHC17
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		if (mode == OperationModes::DATASET)
		{
			vTimestamps[ni]=ttrack;

			// Wait to load the next frame
			double T=0;
			if(ni<totalImages-1)
				T = vTimestamps[ni+1]-tframe;
			else if(ni>0)
				T = tframe-vTimestamps[ni-1];

			if(ttrack<T)
				usleep((T-ttrack)*1e6);
		}
		string file_name = "/home/george/Pictures/test_sequence_laptop/4/starry_night-" + to_string(t1.time_since_epoch().count()) + ".png";
		imwrite(file_name, im);
	}

	// Stop all threads
	SLAM.Shutdown();

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return EXIT_SUCCESS;
}
void parse_arguments(int argc, char* const* argv)
{
	cout << "The following executable can be ran in 2 main mods:" << endl;
	cout << " 0. with live footage from usb camera" << endl;
	cout << " 1. with pre recorded dataset" << endl;
	cout << "The default run option is with usb camera index 0, calibration file TUM1.yaml and original pnp" << endl;
	cout << endl;
	cout << "to choose mode run mode set mode option to 0 or 1, like so: --mode 1" << endl;
	cout << endl;

	int c;

	while (true)
	{
		static struct option long_options[] =
			{
				/* These options don’t set a flag. We distinguish them by their indices. */
				{"mode", required_argument, &mode_int, 'm'},
				{"camera_idx", optional_argument, &camera_idx_int, 'i'},
				{"camera_calib", required_argument, &camera_calib_int, 'c'},
				{"vocabulary", required_argument, &vocabulary_path_int, 'v'},
				{"dataset", optional_argument, &dataset_int, 'd'},
				{"kf_trajectory", optional_argument, &kf_trajectory, 't'},
				{"skip-frames", optional_argument, &skip_frames, 's'},

				/* These options set a flag. */
				{"new_pnp", no_argument, &new_pnp, 1},
				{"verbose", no_argument, &verbose_flag, 1},
				{"brief", no_argument, &verbose_flag, 0},
				{"help", no_argument, nullptr, 'h'},
				{nullptr, 0, nullptr, 0}
			};

		/* getopt_long stores the option index here. */
		int option_index = 0;

		c = getopt_long (argc, argv, "m:i:c:v:d:t:s:h",
			long_options, &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c)
		{
		case 0:
			/* If this option set a flag, do nothing else now. */
			if (long_options[option_index].flag != nullptr)
				break;
			printf ("option %s", long_options[option_index].name);
			if (optarg)
				printf (" with arg %s", optarg);
			printf ("\n");
			break;

		case 'm':
			printf ("option -m with value `%s'\n", optarg);
			mode = stoi(optarg);
			break;

		case 'i':
			printf ("option -i with value `%s'\n", optarg);
			camera_idx = stoi(optarg);
			break;

		case 'c':
			printf ("option -c with value `%s'\n", optarg);
			cameraSettingsPath = optarg;
			break;

		case 'v':
			printf ("option -v with value `%s'\n", optarg);
			vocabularyPath = optarg;
			break;

		case 'd':
			printf ("option -d with value `%s'\n", optarg);
			datasetPath = optarg;
			break;

		case 't':
			printf ("option -t with value `%s'\n", optarg);
			break;

		case 's':
			printf ("option -s with value `%s'\n", optarg);
			break;

		case 'h':
			cout << "--mode				- live camera(0) or prerecorded dataset mode(1)" << endl;
			cout << "--camera_idx		- camera index as set by OS, default value is 0" << endl;
			cout << "--camera_calib		- camera camera calibration file, TUM1.yaml is default" << endl;
			cout << "--vocabulary		- path to orb vocabulary" << endl;
			cout << "--dataset			- path to dataset folder: PATH_TO_SEQUENCE/cam0/" << endl;
			cout << "						cam0 folder should have data folder, and data.csv file" << endl;
			cout << "--new_pnp			- if set new_pnp algorithm for re-localization will be used by ORB_SLAM2" << endl;
			cout << "--kf_trajectory	- key frame trajectory result path" << endl;
			cout << "--skip-frames		- number of frames to skip to check re-localization" << endl;
			cout << endl;
			cout << "verbose			- not implemented, currently only verbose" << endl;
			cout << "brief				- not implemented" << endl;
			cout << endl;
			cout << "Example commands:" << endl;
			cout << "./mono_live_from_usb_cam Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml --camera 0" << endl;
			cout << "./mono_live_from_usb_cam Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml --camera 0" << endl;
			break;
		case '?':
			/* getopt_long already printed an error message. */
			break;

		default:
			abort ();
		}
	}
}

bool has_suffix(const std::string &str, const std::string &suffix)
{
	return str.size() >= suffix.size() &&
		str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

template<typename Out>
void split(const std::string &s, char delimiter, Out result)
{
	std::stringstream ss;
	ss.str(s);
	std::string item;

	while (std::getline(ss, item, delimiter))
		*(result++) = item;
}


std::vector<std::string> split(const std::string &s, char delimiter)
{
	std::vector<std::string> elements;
	split(s, delimiter, std::back_inserter(elements));

	return elements;
}

void LoadImagesEuroc(
	const string &cam0Path,
	vector<string> &vstrImages,
	vector<double> &vTimeStamps,
	int &framesToSkip
)
{
	string strPathTimes;
	string strImagePath;

	if (has_suffix(cam0Path, "/"))
	{
		strPathTimes = cam0Path + "data.csv";
		strImagePath = cam0Path + "data";
	}
	else
	{
		strPathTimes = cam0Path + "/data.csv";
		strImagePath = cam0Path + "/data";
	}

	ifstream fTimes;
	fTimes.open(strPathTimes.c_str());
	vTimeStamps.reserve(5000);
	vstrImages.reserve(5000);

	string s;
	getline(fTimes,s); // discard first line

	while(!fTimes.eof())
	{
		getline(fTimes,s);
		if(!s.empty())
		{
			std::vector<std::string> result = split(s, ',');
			vstrImages.push_back(strImagePath + "/" + result[1]);
			vTimeStamps.push_back(stod(result[0])/1e9);
		}
	}
}
