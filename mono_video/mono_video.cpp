#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <src/fisheye_corrector/fisheye_corrector.h>
using namespace std;

#define usleep(x) Sleep((float)x/1000.0f)

int main(int argc, char **argv)
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
	int nImages = video.get(cv::CAP_PROP_FRAME_COUNT);
	double fps = video.get(cv::CAP_PROP_FPS);
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
	//ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

	float pixel_height = 0.0042;
	float f_image_ = 306.605;
	
	std::string correction_table = argv[4];
	std::cout << "generate corrector" << std::endl;
	std::cout << video.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << video.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
	std::cout << pixel_height << std::endl;
	std::cout << f_image_ << std::endl;


    FisheyeCorrector corrector(correction_table, video.get(cv::CAP_PROP_FRAME_HEIGHT), video.get(cv::CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 40);;

    corrector.setAxisDirection(0, 40, 0);//30,35,-7
    corrector.updateMap();
    corrector.setClipRegion(cv::Rect(cv::Point(0, 475), cv::Point(corrector.getCorrectedSize().width, corrector.getCorrectedSize().height - 500)));
    //correctors[0].setSizeScale(0.5);

	std::cout<<"K:"<<std::endl<<corrector.getIntrinsicMatrix()<<std::endl;

    // Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	std::cout << endl << "-------" << endl;
	std::cout << "Start processing sequence ..." << endl;
	std::cout << "Images in the sequence: " << nImages << endl << endl;

	long vTimeCount = 0;
	// Main loop
	cv::Mat fisheye_im,frame;
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
        corrector.correct(fisheye_im,frame);
        //SLAM.TrackFisheye(fisheye_im, object_class, tframe, correctors);
		SLAM.TrackMonocular(frame, tframe);

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
