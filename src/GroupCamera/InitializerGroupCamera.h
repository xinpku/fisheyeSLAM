#pragma once


#include<opencv2/opencv.hpp>
#include "Frame.h"
#include "Initializer.h"

namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
    class InitializerGroupCamera
    {
    public:
        int mNcameras;
        // Fix the reference frame
        InitializerGroupCamera(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

        // Computes in parallel a fundamental matrix and a homography
        // Selects a model and tries to recover the motion and the structure from motion
        bool Initialize(const Frame& InitialFrame,const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,int& cameraID);


    private:
        std::vector<Initializer> mvInitializers;

    };

} //namespace ORB_SLAM
