#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <src/fisheye_corrector/fisheye_corrector.h>
using namespace std;

#define usleep(x) Sleep((float)x/1000.0f)

int sdfmain(int argc, char **argv)
{
	if (argc != 7)
	{
		cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
		return 1;
	}

	// Retrieve paths to images
	std::string video_path = argv[3];
	std::string object_class_image_path = argv[5];
	bool using_sematic = true;
	if(object_class_image_path=="None")
		using_sematic = false;
	cv::VideoCapture video(video_path);
	int nImages = video.get(CV_CAP_PROP_FRAME_COUNT);
	double fps = video.get(CV_CAP_PROP_FPS);
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::FISHEYE, true);
	//ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

	float pixel_height = 0.0042;
	float f_image_ = 306.605;
	
	std::string correction_table = argv[4];
	std::cout << "generate corrector" << std::endl;
	std::vector<FisheyeCorrector> correctors(3);
	std::cout << video.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << video.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
	std::cout << pixel_height << std::endl;
	std::cout << f_image_ << std::endl;



   correctors[2] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 30, 30);
   correctors[2].setAxisDirection(65, 38, 0);//30,35,-7
   correctors[2].setClipRegion(cv::Rect(cv::Point2f(20, 34), cv::Point2f(correctors[2].getCorrectedSize().width-115, correctors[2].getCorrectedSize().height)));
   //correctors[2].setSizeScale(0.5);
   correctors[2].updateMap();


   std::cout << "*****************************1" << std::endl;
   correctors[1] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 31, 31);
   correctors[1].setAxisDirection(0, 30, 0);//30,35,-7
   //correctors[1].setSizeScale(0.5);
   correctors[1].setClipRegion(cv::Rect(cv::Point2f(0, 0), cv::Point2f(correctors[1].getCorrectedSize().width, correctors[1].getCorrectedSize().height)));
   correctors[1].updateMap();
   std::cout << "*****************************0" << std::endl;
   correctors[0] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 30, 30);
   correctors[0].setAxisDirection(-65, 38, 0);//30,35,-7
   //correctors[0].setSizeScale(0.5);
   correctors[0].setClipRegion(cv::Rect(cv::Point2f(115, 34), cv::Point2f(correctors[0].getCorrectedSize().width-20, correctors[0].getCorrectedSize().height)));
   correctors[0].updateMap();
    /*std::cout << "*****************************3" << std::endl;
    correctors[3] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 30, 30);
    correctors[3].setAxisDirection(-80, -10, 10);//-80, -20, 10
    correctors[3].updateMap();
    correctors[3].setClipRegion(cv::Rect(cv::Point2f(150, 100), cv::Point2f(correctors[3].getCorrectedSize().width, correctors[3].getCorrectedSize().height)));
    //correctors[0].setSizeScale(0.5);

    std::cout << "*****************************4" << std::endl;
    correctors[4] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 30, 50);
    correctors[4].setAxisDirection(0, -20, 0);//0, -30, 0
    correctors[4].updateMap();
    correctors[4].setClipRegion(cv::Rect(cv::Point2f(0, 0), cv::Point2f(correctors[4].getCorrectedSize().width, correctors[4].getCorrectedSize().height)));
    //correctors[1].setSizeScale(0.5);
    std::cout << "*****************************5" << std::endl;
    correctors[5] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 30, 30);
    correctors[5].setAxisDirection(80, -10, -10);//80, -20, -10
    correctors[5].updateMap();
    correctors[5].setClipRegion(cv::Rect(cv::Point2f(0, 100), cv::Point2f(correctors[5].getCorrectedSize().width-150, correctors[5].getCorrectedSize().height)));
    //correctors[2].setSizeScale(0.5);*/

    // Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	std::cout << endl << "-------" << endl;
	std::cout << "Start processing sequence ..." << endl;
	std::cout << "Images in the sequence: " << nImages << endl << endl;

	long vTimeCount = 0;
	// Main loop
	cv::Mat fisheye_im;     
	std::stringstream sst;
	sst << argv[6];
	int start_frame;
	sst >> start_frame;
    std::cout<<"start_frame"<<start_frame<<std::endl;
	for (int ni = 0; ni<nImages; ni++)
	{
		
		// Read image from file
		video >> fisheye_im;
		if (ni < start_frame)
			continue;
		cv::cvtColor(fisheye_im, fisheye_im, cv::COLOR_BGR2GRAY);
		double tframe = vTimeCount;

		if (fisheye_im.empty())
		{
			cerr << endl << "Failed to load image at: " << vTimeCount << endl;
			return 1;
		}
		cv::Mat object_class;
		if(using_sematic)
		{
			sst.clear();
			sst.str("");
			sst << object_class_image_path << "/" << setfill('0') << setw(5) << ni << ".png";
			//std::cout<<sst.str()<<std::endl;
			object_class = cv::imread(sst.str(), -1);
			cv::resize(object_class, object_class, cv::Size(1280, 720), 0, 0, cv::INTER_NEAREST);
		}
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

		// Pass the image to the SLAM system
        SLAM.TrackFisheye(fisheye_im, object_class, tframe, correctors);
		//SLAM.TrackMonocular(imgs[0], tframe);

		if (SLAM.GetTrackingState() == 3 || (ni>=30&&ni%30==0))
		{
			SLAM.SaveMapClouds("pointClouds.vtx");
			SLAM.SaveTrajectoryVtx("CameraTrajectory.vtx");
		}
        //std::cout<<"ni "<<ni<<std::endl;
		//cv::waitKey(0);
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;
		vTimeCount += 1;//1000.0f / fps;
		// Wait to load the next frame
		cv::waitKey(10);
	}
// Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory_all.txt");
    SLAM.SaveTrajectoryVtx("CameraTrajectory.vtx");
    SLAM.SaveKeyFrameTrajectoryTUM("CameraTrajectory_keyframe.txt");
    SLAM.SaveMapClouds("pointClouds.vtx");
    //cv::waitKey(0);
	// Stop all threads
	std::cout<<"keyframe size "<<SLAM.mpMap->GetAllKeyFrames().size()<<std::endl;
	SLAM.SaveKeyframes(SLAM.mpMap->GetAllKeyFrames(),"keyframes.map");
    SLAM.SaveMapPoints(SLAM.mpMap->GetAllMapPoints(),"mapPoints.map");
	SLAM.Shutdown();

	// Tracking time statistics
	std::sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	std::cout << "-------" << endl << endl;
	std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	std::cout << "mean tracking time: " << totaltime / nImages << endl;


	return 0;
}
