#include "planeFit.h"
static const int minimalSetSize = 4;
static double inlierThreshold = 1e-2;
const double confidence = 0.99999;
const double inlierRatio = 0.3;
std::vector<int> randomPick(int sum, int num)
{
    if (num > sum)return std::vector<int>();
    std::random_device rd;

    std::vector<int> list(sum), result(num);
    int curSize = sum;
    for (int i = 0; i<sum; i++)
    {
        list[i] = i;
    }
    for (int i = 0; i<num; i++)
    {
        int curPose = rd() % curSize;

        result[i] = list[curPose];
        std::swap(list[curSize - 1], list[curPose]);
        curSize--;
    }
    return result;
}

Eigen::Vector4d planeFit(const Eigen::MatrixXd& point_cloud_matrix)
{
    unsigned int  row_num, col_num;
    row_num = point_cloud_matrix.rows();
    col_num = point_cloud_matrix.cols();

    if (col_num != 3 )
    {
        std::cout << "The matrix is not required!" << std::endl;
    }

    Eigen::MatrixXd  point_mean_matrix = Eigen::MatrixXd::Zero(row_num, col_num);
    double x_mean, y_mean, z_mean;
    x_mean = point_cloud_matrix.col(0).mean();
    y_mean = point_cloud_matrix.col(1).mean();
    z_mean = point_cloud_matrix.col(2).mean();

    for (unsigned int i = 0; i < row_num; i++)
    {
        point_mean_matrix(i, 0) = x_mean;  point_mean_matrix(i, 1) = y_mean;  point_mean_matrix(i, 2) = z_mean;
    }

    Eigen::MatrixXd  point_delta_matrix = point_cloud_matrix - point_mean_matrix;

    Eigen::MatrixXd  matrix_A = point_delta_matrix.transpose() * point_delta_matrix;

    Eigen::EigenSolver<Eigen::Matrix3d>  eigensolver(matrix_A);
    //std::cout << "The eigenvalues matrix_A is :" << std::endl << eigensolver.eigenvalues() << std::endl;
    //std::cout << "The eigenvetctors matrix_A is :" << std::endl << eigensolver.eigenvectors()  << std::endl;

    Eigen::Matrix3d eigvalues = eigensolver.pseudoEigenvalueMatrix();
    Eigen::Matrix3d eigvectors = eigensolver.pseudoEigenvectors();

    double min_eigvalue = eigvalues(0, 0);
    unsigned int min_pos = 0;
    for (unsigned int i = 1; i < col_num; i++)
    {
        if (min_eigvalue >= eigvalues(i, i))
        {
            min_eigvalue = eigvalues(i, i);
            min_pos = i;
        }
    }

    Eigen::Vector4d  plane_coef(eigvectors(0, min_pos), eigvectors(1, min_pos), eigvectors(2, min_pos), 0);
    plane_coef(3) = -(plane_coef(0) * x_mean + plane_coef(1) * y_mean + plane_coef(2) * z_mean);

    return plane_coef;
}




int RANSAConce(Eigen::Vector4d& _currentResult,const Eigen::MatrixXd& point_cloud_matrix)
{
    //Make a minimal set of points randomly

    std::vector<int> pos = randomPick(point_cloud_matrix.rows(), minimalSetSize);
    if (pos.size() == 0) return 0;//Return false if can't make a set;


    Eigen::MatrixXd minimalPointsSet1(minimalSetSize,3);
    for (int i = 0; i<minimalSetSize; i++)
    {
        //std::cout<<"pos[i] "<<pos[i]<<" i "<<i<<std::endl;
        minimalPointsSet1.row(i) = point_cloud_matrix.row(pos[i]);
        //std::cout<<"end"<<std::endl;
    }

    _currentResult = planeFit(minimalPointsSet1);
    double avg_distance = 0;
    for (int i = 0; i<minimalSetSize; i++)
    {
        avg_distance += distance_point_plane(_currentResult,minimalPointsSet1.row(i));
    }
    avg_distance/=minimalSetSize;
    inlierThreshold = 2*avg_distance;
    std::cout<<"plane fit  "<<inlierThreshold<<std::endl;
    int inlierCount = 0;

    for(int i = 0;i<point_cloud_matrix.rows();i++)
    {
        Eigen::Vector3d point = point_cloud_matrix.row(i);
        if(distance_point_plane(_currentResult,point) < inlierThreshold)
        {
            inlierCount++;
        }
    }

    return inlierCount;
}



Eigen::Vector4d findPlaneCoeff(const Eigen::MatrixXd& point_cloud_matrix,std::vector<bool>& inlierState,int& inlier_num)
{
    int maxSupportNumber = 0;

    int iterationNumber = std::round(std::log10(1 - confidence) / std::log10(1 - pow(inlierRatio, minimalSetSize)));
    //std::cout<<"iterationNumber "<<iterationNumber<<std::endl;
    inlierState.resize(point_cloud_matrix.rows(), false);
    Eigen::Vector4d currentResult,result;
    for (int i = 0; i<iterationNumber; i++)
    {
        int supportNumber = RANSAConce(currentResult,point_cloud_matrix);
        //std::cout << supportNumber << std::endl;
        if (supportNumber>maxSupportNumber)
        {
            maxSupportNumber = supportNumber;
            result = currentResult;
        }
    }
    inlier_num = maxSupportNumber;
    std::cout<<"-----inliers/points "<<inlier_num<<"/"<<point_cloud_matrix.rows()<<std::endl;
    if((float)inlier_num/point_cloud_matrix.rows()<0.7)
        return Eigen::Vector4d();

    for(int i = 0;i<point_cloud_matrix.rows();i++)
    {
        Eigen::Vector3d point = point_cloud_matrix.row(i);
        if(distance_point_plane(result,point) < inlierThreshold)
        {
            inlierState[i] = true;
        }
    }

    Eigen::MatrixXd point_cloud_inlier(maxSupportNumber,3);
    for (int i = 0, j = 0; i < maxSupportNumber&&j<inlierState.size(); j++)
    {
        if (inlierState[j])
        {
            point_cloud_inlier.row(i) = point_cloud_matrix.row(j);
            i++;
        }
    }

    result = planeFit(point_cloud_inlier);
    return result;
}

