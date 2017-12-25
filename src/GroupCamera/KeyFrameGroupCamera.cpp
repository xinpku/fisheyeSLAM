#include "KeyFrame.h"


namespace ORB_SLAM2
{
    //Functions related to groupCamera
    void KeyFrame::UpdateMultiCameraPose()
    {
        for (int i = 0; i < mvTcg.size(); i++)
        {
            mvTcwSubcamera[i] = mvTcg[i] * Tcw;
            mvOwSubcamera[i] = -(mvTcwSubcamera[i].rowRange(0, 3).colRange(0, 3)).t()*(mvTcwSubcamera[i].rowRange(0, 3).col(3));
        }
    }
    std::vector<cv::KeyPoint> KeyFrame::getKeypointUnSubCamera(int c) const
    {
        int start_pos = kp_start_pos[c];
        int end_pose = (c==Ncameras-1?mvKeysUn.size():kp_start_pos[c+1]);
        std::vector<cv::KeyPoint> keypoints;
        keypoints.reserve(end_pose - start_pos);
        for(int i = start_pos;i<end_pose;i++)
        {
            keypoints.push_back(mvKeysUn[i]);
        }
        return keypoints;
    }
    cv::Mat KeyFrame::getTcwSubCamera(int i)
    {
        return mvTcwSubcamera[i].clone();
    }
    cv::Mat KeyFrame::getCameraCenterSubCamera(int i)
    {
        return mvOwSubcamera[i].clone();
    }
    cv::Mat KeyFrame::GetRotationSubCamera(int i)
    {
        unique_lock<mutex> lock(mMutexPose);
        return mvTcwSubcamera[i].rowRange(0, 3).colRange(0, 3).clone();
    }
    cv::Mat KeyFrame::GetTranslationSubCamera(int i)
    {
        unique_lock<mutex> lock(mMutexPose);
        return mvTcwSubcamera[i].rowRange(0, 3).col(3).clone();
    }
}