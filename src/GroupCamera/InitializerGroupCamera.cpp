#include "InitializerGroupCamera.h"
#include "Initializer.h"
#include<thread>
#include "debug_utils/debug_utils.h"
namespace ORB_SLAM2
{
    InitializerGroupCamera::InitializerGroupCamera(const Frame &ReferenceFrame, float sigma, int iterations)
    {
        mNcameras =ReferenceFrame.Ncameras;
        for(int c = 0;c<ReferenceFrame.Ncameras;c++)
        {

            Initializer initializer =  Initializer(ReferenceFrame.mK,ReferenceFrame.mvKeysUn,1.0,200);
            mvInitializers.push_back(initializer);
        }
    }

    bool InitializerGroupCamera::Initialize(const Frame& InitialFrame,const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,int& cameraID)
    {


        for(int c = 0;c<mNcameras;c++)
        {
            int start_pos = InitialFrame.kp_start_pos[c];
            int end_pose = (c==mNcameras-1?InitialFrame.mvKeysUn.size():InitialFrame.kp_start_pos[c+1]);


            int start_pos2 = CurrentFrame.kp_start_pos[c];
            int end_pose2 = (c==mNcameras-1?CurrentFrame.mvKeysUn.size():CurrentFrame.kp_start_pos[c+1]);

            std::vector<int> vMatches(vMatches12.size(),-1);

            for(int i = start_pos;i<end_pose;i++)
            {
                if((vMatches12[i]>=start_pos2&&vMatches12[i]<end_pose2))
                    vMatches[i] = vMatches12[i];
            }

            if(mvInitializers[c].Initialize(CurrentFrame.mvKeysUn,vMatches,R21,t21,vP3D,vbTriangulated))
            {
                cameraID =c;

                cv::Mat T21 = cv::Mat::eye(4,4,CV_32F);
                R21.copyTo(T21.rowRange(0,3).colRange(0,3));
                t21.copyTo(T21.rowRange(0,3).col(3));
                T21 = CurrentFrame.mvTgc[c]*T21*CurrentFrame.mvTcg[c];

                R21 = T21.rowRange(0,3).colRange(0,3);
                t21 = T21.rowRange(0,3).col(3);
                for(int i = 0;i<vP3D.size();i++)
                {
                    if(vbTriangulated[i])
                    {
                        cv::Mat point = cv::Mat::ones(4,1,CV_32F);
                        cv::Mat(vP3D[i]).copyTo(point.rowRange(0,3));
                        cv::Mat p = CurrentFrame.mvTgc[c]*point;
                        vP3D[i] =cv::Point3f(p.rowRange(0,3));
                    }

                }
                //TO DO
                //Initialize other camera using given pose
                return true;
            }
        }

        return false;
    }




    Initializer::Initializer(const cv::Mat &K,const std::vector<cv::KeyPoint>& vKeys, float sigma, int iterations)
    {
        mK = K.clone();

        mvKeys1 = vKeys;

        mSigma = sigma;
        mSigma2 = sigma*sigma;
        mMaxIterations = iterations;
    }


    bool Initializer::Initialize(const std::vector<cv::KeyPoint>& vKeys, const vector<int> &vMatches12,
                                 cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
    {
        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        mvKeys2 = vKeys;
        print_value(mvKeys2.size())
        mvMatches12.clear();
        mvMatches12.reserve(mvKeys2.size());
        mvbMatched1.resize(mvKeys1.size());
        for(size_t i=0, iend=vMatches12.size();i<iend; i++)
        {
            if(vMatches12[i]>=0)
            {
                mvMatches12.push_back(make_pair(i,vMatches12[i]));
                mvbMatched1[i]=true;
            }
            else
                mvbMatched1[i]=false;
        }

        const int N = mvMatches12.size();

        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }

        // Generate sets of 8 points for each RANSAC iteration
        mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

        DUtils::Random::SeedRandOnce(0);

        for(int it=0; it<mMaxIterations; it++)
        {
            vAvailableIndices = vAllIndices;

            // Select a minimum set
            for(size_t j=0; j<8; j++)
            {
                int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
                int idx = vAvailableIndices[randi];

                mvSets[it][j] = idx;

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }
        }

        // Launch threads to compute in parallel a fundamental matrix and a homography
        vector<bool> vbMatchesInliersH, vbMatchesInliersF;
        float SH, SF;
        cv::Mat H, F;

        std::thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
        std::thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

        // Wait until both threads have finished
        threadH.join();
        threadF.join();

        // Compute ratio of scores
        float RH = SH/(SH+SF);


        if(SH+SF==0)
            return false;

        // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
       if(RH>0.40)
            return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
        else //if(pF_HF>0.6)*/
            return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,0.15,30);

        return false;
    }

}