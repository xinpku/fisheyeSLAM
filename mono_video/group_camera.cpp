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

    std::string settingFile = argv[2];
    cv::FileStorage fSettings(settingFile.c_str(), cv::FileStorage::READ);
    int Ncameras = fSettings["GroupCamera.n_frame"];

    // Retrieve paths to images
    std::string video_path = argv[3];

    std::vector<std::string> video_name(Ncameras);
    std::vector<cv::VideoCapture> videos(Ncameras);

    for(int i = 0;i<Ncameras;i++)
    {
        std::stringstream videoName;
        videoName<<"GroupCamera.VideoName."<<i;
        video_name[i] = std::string(fSettings[videoName.str()]);

        videos[i].open(video_path+video_name[i]);
    }



    std::string object_class_image_path = argv[5];
    bool using_sematic = true;
    if(object_class_image_path=="None")
        using_sematic = false;



    int nImages = videos[0].get(CV_CAP_PROP_FRAME_COUNT);
    double fps = videos[0].get(CV_CAP_PROP_FPS);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::GROUPCAMERA, true);


    float pixel_height = 0.0042;
    float f_image_ = 306.605;

    std::string correction_table = argv[4];
    std::cout << "generate corrector" << std::endl;
    std::cout << videos[0].get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << videos[0].get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << pixel_height << std::endl;
    std::cout << f_image_ << std::endl;

    FisheyeCorrector corrector(correction_table, videos[0].get(CV_CAP_PROP_FRAME_HEIGHT), videos[0].get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 60);;

    corrector.setAxisDirection(0, 0, 0);//30,35,-7
    //corrector.setSizeScale(0.5);
    corrector.setClipRegion(cv::Rect(cv::Point2f(0, 0), cv::Point2f(corrector.getCorrectedSize().width, corrector.getCorrectedSize().height)));
    corrector.updateMap();

    std::cout<<corrector.getIntrinsicMatrix()<<std::endl;
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout << endl << "-------" << endl;
    std::cout << "Start processing sequence ..." << endl;
    std::cout << "Images in the sequence: " << nImages << endl << endl;

    long vTimeCount = 0;
    // Main loop
    std::vector<cv::Mat> fisheye_ims(Ncameras);
    std::vector<cv::Mat> ims(Ncameras);
    std::stringstream sst;
    sst << argv[6];
    int start_frame;
    sst >> start_frame;
    std::cout<<"start_frame"<<start_frame<<std::endl;
    for (int ni = 0; ni<nImages; ni++)
    {

        // Read image from file
        for(int c =0;c<Ncameras;c++)
        {
            videos[c] >> fisheye_ims[c];
            if (fisheye_ims[c].empty())
            {
                cerr << endl << "Failed to load image at: " << vTimeCount << endl;
                return 1;
            }

            cv::cvtColor(fisheye_ims[c], fisheye_ims[c], cv::COLOR_BGR2GRAY);
            corrector.correct(fisheye_ims[c],ims[c]);
        }


        if (ni < start_frame)
            continue;


        double tframe = vTimeCount;


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
        SLAM.TrackGroupCamera(ims,tframe);


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
