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