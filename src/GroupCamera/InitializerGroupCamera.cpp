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

            Initializer initializer =  Initializer(ReferenceFrame.mvK[c],ReferenceFrame.mvKeysUn,1.0,200);
            mvInitializers.push_back(initializer);
        }
    }

    bool InitializerGroupCamera::Initialize(const Frame& InitialFrame,const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,int& cameraID)
    {
printOFF;
        bool init_success = false;
        cv::Mat T21 = cv::Mat::eye(4,4,CV_32F);

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
                init_success = true;
                break;
            }
        }


        if(init_success == false)
            return false;

//Reconstruct for other cameras
        for(int c = 0;c<mNcameras;c++)
        {
            if(c==cameraID)
                continue;

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

            cv::Mat T21_c = CurrentFrame.mvTcg[c]*T21*CurrentFrame.mvTgc[c];
            cv::Mat R21_c = T21_c.rowRange(0,3).colRange(0,3);
            cv::Mat t21_c = T21_c.rowRange(0,3).col(3);
            print_value(c)
           int add_point_size = mvInitializers[c].InitializeWithGivenPose(InitialFrame,CurrentFrame,vMatches,R21_c,t21_c,vP3D,vbTriangulated,c);

            int add_count = 0;
            for(int i = start_pos;i<end_pose;i++)
            {
                if(vbTriangulated[i])
                {
                    cv::Mat point = cv::Mat::ones(4,1,CV_32F);
                    cv::Mat(vP3D[i]).copyTo(point.rowRange(0,3));
                    cv::Mat p = CurrentFrame.mvTgc[c]*point;
                    vP3D[i] =cv::Point3f(p.rowRange(0,3));
                    add_count ++;
                }
            }
            print_value(add_count);

        }

        return true;
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




    cv::Mat SkewSymmetricMatrix(const cv::Mat &v)
    {
        return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2),               0,-v.at<float>(0),
                -v.at<float>(1),  v.at<float>(0),              0);
    }

    cv::Mat ComputeF12(const cv::Mat& R12,const cv::Mat& t12,const cv::Mat& K1,const cv::Mat& K2)
    {
        cv::Mat t12x = SkewSymmetricMatrix(t12);

        return K1.t().inv()*t12x*R12*K2.inv();
    }


    int Initializer::InitializeWithGivenPose(const Frame &LastFrame,const Frame& CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                                              vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,int cameraID)
    {
        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        mvKeys1 = LastFrame.mvKeysUn;
        mvKeys2 = CurrentFrame.mvKeysUn;

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
        cv::Mat F21 = ComputeF12(R21,t21,mK,mK);

        CheckFundamental(F21, vbMatchesInliersF, mSigma);
        // Calibration parameters
        const float fx = mK.at<float>(0,0);
        const float fy = mK.at<float>(1,1);
        const float cx = mK.at<float>(0,2);
        const float cy = mK.at<float>(1,2);


        vector<float> vCosParallax;
        vCosParallax.reserve(mvKeys1.size());


        // Camera 1 Projection Matrix K[I|0]


        cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
        mK.copyTo(P1.rowRange(0,3).colRange(0,3));
        cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3,4,CV_32F);
        R21.copyTo(P2.rowRange(0,3).colRange(0,3));
        t21.copyTo(P2.rowRange(0,3).col(3));
        P2 = mK*P2;
        cv::Mat O2 = -R21.t()*t21;


        int nGood=0;
        int nTrigulated = 0;
        for(size_t i=0, iend=mvMatches12.size();i<iend;i++)
        {
            if(!vbMatchesInliersF[i]||!mvbMatched1[i])
                continue;
            //std::cout<<mvMatches12[i].first <<" "<<mvMatches12[i].second<<"v"<<mvKeys1.size()<<" "<<mvKeys2.size()<<std::endl;
            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];
            cv::Mat p3dC1;

            //std::cout<<kp1.pt<<std::endl<<kp2.pt<<std::endl;
            Triangulate(kp1,kp2,P1,P2,p3dC1);

            if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
            {
                vbTriangulated[mvMatches12[i].first]=false;
                continue;
            }

            // Check parallax
            cv::Mat normal1 = p3dC1 - O1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = p3dC1 - O2;
            float dist2 = cv::norm(normal2);

            float cosParallax = normal1.dot(normal2)/(dist1*dist2);


            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            cv::Mat p3dC2 = R21*p3dC1+t21;
            p3dC1 = p3dC1;

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)

            if(p3dC1.at<float>(2)<=0)
                continue;
            if(p3dC2.at<float>(2)<=0)
                continue;

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1.at<float>(2);
            im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
            im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

            float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

            if(squareError1>4.0*mSigma2)
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2.at<float>(2);
            im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
            im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

            float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

            if(squareError2>4.0*mSigma2)
                continue;

            vCosParallax.push_back(cosParallax);
            vP3D[mvMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
            nGood++;
            //std::cout<<"cosParallax "<<cosParallax<<" cosParallax<0.99998 " <<(cosParallax<0.99998)<<std::endl;
            if(cosParallax<0.999998)
            {
                vbTriangulated[mvMatches12[i].first]=true;
                nTrigulated++;
            }
        }

        int Ninlier=0;
        for(size_t i=0, iend = vbMatchesInliersF.size() ; i<iend; i++)
            if(vbMatchesInliersF[i])
                Ninlier++;

        int nMinGood = max(static_cast<int>(0.1*Ninlier),30);
        print_value(Ninlier)
        print_value(nGood)
        print_value(nTrigulated)
        return nTrigulated;
    }




}