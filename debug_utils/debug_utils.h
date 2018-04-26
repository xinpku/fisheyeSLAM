#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <map>

#define PI 3.1415926

class CameraTrajectory;

extern cv::Mat result;
extern cv::Mat z_result;
extern double projectPoint_z;

extern CameraTrajectory GTtrajectory;




#define flushlog DebugLog::getLogFile()->flushLog();
#define debugLog(v) DebugLog::getLogFile()->wirteLog(v);flushlog;//DebugLog::getLogFile()->wirteLog(" ")
#define debugLogLine(v) debugLog(v);debugLog("\n");flushlog;
#define debugWriteLog(v) debugLog(#v);debugLog(":\n")debugLog(v);debugLog("\n");flushlog;
#define debugDisp(x) std::cout<<#x<<std::endl<<x<<std::endl;//log(#x);log("\n");log(x);log("\n");
#define debugLogMatrix(v,row,col) for(int i = 0;i<row;i++){ for(int j = 0;j<col-1;j++){debugLog(v[i*col+j]);debugLog("\t");debugLog("\t");}debugLog(v[i*col+col-1]);}debugLog("\n");flushlog;


	class DebugLog
	{
	private:
		DebugLog()//����ģʽ
		{
			File.open("log.txt");
		};


	public:
		std::ofstream File;
		template<class T>
		void wirteLog(std::vector<T> v)
		{


			for (int i = 0; i < v.size(); i++)
			{
				File << v[i] << std::endl;
			}
		}


		template<class T>
		void wirteLog(T v)
		{
			File << v;// << std::endl;
		}


		void flushLog()
		{
			File.flush();
		}


		static DebugLog* getLogFile()//getInstance()
		{
			static DebugLog* instance;
			if (instance == NULL)
			{
				instance = new DebugLog();
			}
			return instance;
		}




	};


class PrintInfo
{
public:
	static bool print_info;
	static int print_depth;
	static int need_print_ID_global;
	static int current_ID_global;
	bool real_deconstruct = true;
	static std::map<std::string,int> print_flags;
	PrintInfo()
	{
		PrintInfo::print_info = true;

		PrintInfo::print_depth++;
	}

	PrintInfo(int current_ID)
	{
		PrintInfo::current_ID_global = current_ID;
		real_deconstruct = false;
	}

	~PrintInfo()
	{
        if(real_deconstruct)
        {
            if(PrintInfo::printTest())
            {
                for(int i = 0;i<PrintInfo::print_depth;i++)std::cout<<"    ";
                std::cout<<"end print ------------"<<std::endl<<std::endl;
            }

            PrintInfo::print_depth--;
			if(PrintInfo::print_depth<=-1)
				PrintInfo::print_info = false;
        }
        else
        {
            PrintInfo::current_ID_global = -999;
        }
    }

	static bool printTest()
	{
		if(PrintInfo::print_info&&PrintInfo::processingTest())
			return true;
		else
			return false;
	}

    static bool processingTest()
    {
        if((PrintInfo::need_print_ID_global==-999||PrintInfo::current_ID_global==-999||PrintInfo::need_print_ID_global==PrintInfo::current_ID_global))
            return true;
        else
            return false;
    }
};

class PrintCloser
{
public:
    bool old_print_state = false;
    PrintCloser()
    {
        old_print_state = PrintInfo::print_info;
        PrintInfo::print_info = false;
        if(old_print_state)
        {
            for(int i = 0;i<PrintInfo::print_depth;i++)std::cout<<"    ";
            std::cout<<"print OFF"<<std::endl;
        }

    }

    ~PrintCloser()
    {
        PrintInfo::print_info = old_print_state;
    }

};



std::string type2str(int type);


#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#define PI 3.1415926
#define printSkip std::cout<<"  ";for(int i = 0;i<PrintInfo::print_depth;i++)std::cout<<"    ";
#define printON PrintInfo PRINT_INFO;if(PrintInfo::processingTest()){if(PrintInfo::print_depth>0)std::cout<<std::endl;for(int i = 0;i<PrintInfo::print_depth;i++)std::cout<<"    ";std::cout<<"begin print------------("<<__FUNCTION__<<")"<<std::endl;};
#define printOFF if(PrintInfo::printTest()){printSkip;std::cout<<"("<<__FUNCTION__<<__LINE__<<") :";};PrintCloser PRINT_CLOSER;
#define printSetCurrentID(id) PrintInfo PRINT_INFO_SET_ID(id);
#define print_value(x,force_output...)  if(PrintInfo::printTest(),##force_output){printSkip;std::cout<<#x<<": "<<x<<"   ("<<__FUNCTION__<<__LINE__<<")"<<std::endl;};
#define print_string(x,force_output...)  if(PrintInfo::printTest(),##force_output){printSkip;std::cout<<x<<"   ("<<__FUNCTION__<<__LINE__<<")"<<std::endl;};
#define print_vect_cv(x,force_output...) if(PrintInfo::printTest(),##force_output){printSkip;std::cout<<#x<<": "<<x.t()<<std::endl;};
#define print_vect_eigen(x,force_output...) if(PrintInfo::printTest(),##force_output){printSkip;std::cout<<#x<<": "<<x.transpose()<<"   ("<<__FUNCTION__<<__LINE__<<")"<<std::endl;};
#define print_mat(x,force_output...) if(PrintInfo::printTest(),##force_output){printSkip;std::cout<<"("<<__FUNCTION__<<__LINE__<<") "<<#x<<":"<<std::endl<<x<<std::endl;};
#define print_vector(x,force_output...) if(PrintInfo::printTest(),##force_output){printSkip;std::cout<<std::endl<<#x<<": "<<std::endl;for(int i = 0;i<x.size();i++){std::cout<<x[i]<<" ";}std::cout<<std::endl<<std::endl;};
#define print_array(x,s,force_output...) if(PrintInfo::printTest(),##force_output){printSkip; std::cout<<#x<<": "<<std::endl;for(int i = 0;i<s;i++){printSkip;std::cout<<x[i]<<" ";}std::cout<<std::endl;}

#define log_value(x,file_name) std::ofstream file(file_name);file<<#x<<": "<<x<<std::endl;
#define log_vect_cv(x,file_name) std::ofstream file(file_name);file<<#x<<": "<<x.t()<<std::endl;
#define log_mat(x,file_name) std::ofstream file(file_name);file<<#x<<":"<<std::endl<<x<<std::endl;
#define log_vector(x) std::ofstream file(file_name);file<<#x<<": "<<std::endl;for(int i = 0;i<x.size();i++){file<<x[i]<<" ";}file<<std::endl;

#define imshow_cv(x) cv::namedWindow(#x,0);cv::imshow(#x,x);

inline cv::Mat eigenMatrixToCvMat(const Eigen::MatrixXd& eigen_matrix)
{
	cv::Mat cv_matrix;
	//Eigen::Matrix3f eigen_matrix = eigen_matrix_d.cast<float>();
	if (eigen_matrix.IsRowMajor)
		cv_matrix = cv::Mat(eigen_matrix.rows(), eigen_matrix.cols(), CV_64F);
	else
		cv_matrix = cv::Mat(eigen_matrix.cols(), eigen_matrix.rows(), CV_64F);

	cv_matrix.data = (uchar*)eigen_matrix.data();
	if (!eigen_matrix.IsRowMajor)
		cv_matrix = cv_matrix.t();

	cv_matrix.convertTo(cv_matrix,CV_32F);
	return cv_matrix.clone();
}
inline Eigen::MatrixXd cvMatToEigenMatrix(const cv::Mat& cv_matrix)
{
	Eigen::MatrixXf matrix = Eigen::Map<Eigen::MatrixXf>((float*)cv::Mat(cv_matrix.t()).data, cv_matrix.rows, cv_matrix.cols);
	return matrix.cast<double>();
}

inline cv::Mat makeHomoVector(const cv::Mat& point)
{
	/*cv::Mat point_homo = cv::Mat(point.rows+1,point.cols,CV_32F);
    float* point_homo_data = (float*)point_homo.data;
    float* point_data = (float*)point.data;
	for(int i = 0;i<point.rows;i++)
    {
    }*/
	cv::Mat point_homo = point.clone();
	point_homo.resize(point.rows+1);
	point_homo.at<float>(point.rows) = 1;
	return point_homo.clone();
}

template <class T>
inline Eigen::Matrix<T,4,1> makeHomoVector(const Eigen::Matrix<T,3,1>& point)
{
    Eigen::Matrix<T,4,1> homo;
    homo<<point,T(1);
    return homo;
}

inline double gaussianProb(double x,double mu,double sig)
{
	return (1 / (sqrt(2 * PI)*sig))*exp(-((x - mu)*(x - mu)) / (2 * sig*sig));
}
