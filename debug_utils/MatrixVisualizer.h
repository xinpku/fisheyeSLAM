#pragma once
#include "debug_utils.h"
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <opencv2/core/core.hpp>

#define MatVis_INT_ARRAY MatVis::Vis()->intBuf
#define MatVis_FLOAT_ARRAY MatVis::Vis()->floatBuf
#define MatVis_DOUBLE_ARRAY MatVis::Vis()->doubleBuf


#define makeIntVisArray(v,member,w,h) MatVis_INT_ARRAY = (int*)malloc(sizeof(int)*w*h);for (int i = 0; i < h; i++){for (int j = 0; j < w; j++){MatVis_INT_ARRAY[i*w + j] = v[i*w + j].##member; }}
#define makeFloatVisArray(v,member,w,h) MatVis_FLOAT_ARRAY = (float*)malloc(sizeof(float)*w*h);for (int i = 0; i < h; i++){for (int j = 0; j < w; j++){MatVis_FLOAT_ARRAY[i*w + j] = v[i*w + j].##member; }}
#define makeDouleVisArray(v,member,w,h) MatVis_DOUBLE_ARRAY = (double*)malloc(sizeof(double)*w*h);for (int i = 0; i < h; i++){for (int j = 0; j < w; j++){MatVis_DOUBLE_ARRAY[i*w + j] = v[i*w + j].##member; }}


#define VisualIntArrayMember(v,member,w,h) MatVis::Vis()->lock();makeIntVisArray(v,member,w,h);MatVis::Vis()->toString(MatVis_INT_ARRAY, width, height);MatVis::Vis()->unlock();
#define VisualFloatArrayMember(v,member,w,h) MatVis::Vis()->lock();makeFloatVisArray(v,member,w,h);MatVis::Vis()->toString(MatVis_FLOAT_ARRAY, width, height);MatVis::Vis()->unlock();
#define VisualDoubleArrayMember(v,member,w,h) MatVis::Vis()->lock();makeDouleVisArray(v,member,w,h);MatVis::Vis()->toString(MatVis_DOUBLE_ARRAY, width, height);MatVis::Vis()->unlock();
//catMember(v[i*w + j], IDEPTH)




#define MAXSETW 15
class MatVis
{


private:
	std::mutex mutex;
	MatVis()
	{
	};


public:
	
	std::stringstream transfer;
	std::string text;
	float* floatBuf;
	int* intBuf;
	double* doubleBuf;
	template<class T>
	std::string toString(const T v);
	template<class T>
	std::string toString(const T v, int w, int h);


	//template<class T>
	//std::string toStringCurrent(T v)
	//{
	//	transfer << v;
	//}


	std::string VisIntArray(const int* v, int w, int h);


	std::string VisUcharArray(const unsigned char* v, int w, int h);
	
	std::string VisShortArray(const short* v, int w, int h);



	std::string VisFloatArray(const float* v, int w, int h);
	


	std::string VisDoubleArray(const double* v, int w, int h);
	


	std::string VisEigen(const Eigen::Matrix3f& v);
	
	std::string VisEigen(const Eigen::Matrix4f& v);
	


	std::string VisEigen(const Eigen::Vector3f& v);
	


	std::string VisEigen(const Eigen::VectorXf& v);
	




	std::string VisEigenVec2(const Eigen::Vector2f* v, int w, int h, int k);
	


	std::string VisEigenVec3(const Eigen::Vector3f* v, int w, int h, int k);
	




	std::string VisEigenVec4(const Eigen::Vector4f* v, int w, int h, int k);
	


	std::string VisCVMat(const cv::Mat& v);
	
	std::string VisCVMatType(const cv::Mat& v);


	static MatVis* Vis();//getInstance()
	


	void logToFile(char* fileName);
	


	void lock();
	
	void unlock();
	
};