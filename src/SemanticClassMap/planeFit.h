#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
//There are too many parameters in the parameter list if design the RANSAC as a function, so I design it as a template class.
//Another advantage of designing as a class is that instead of having to declare many variables, such as result matrix and state vector, when you call a RANSAC function, you only need to declare a RANSAC class.


std::vector<int> randomPick(int sum, int num);
Eigen::Vector4d findPlaneCoeff(const Eigen::MatrixXd& point_cloud_matrix,std::vector<bool>& inlierState,int& inlier_num);

inline double distance_point_plane(const Eigen::Vector4d& plane,const Eigen::Vector3d& point)
{
    Eigen::Vector4d point_homo;
    Eigen::Vector3d plane_normal = plane.block(0,0,3,1);
    point_homo<<point,1;
    return std::abs(plane.dot(point_homo)/plane_normal.norm());
};
