#include "MatrixVisualizer.h"




template<class T>
std::string MatVis::toString(const T v)
{
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << v;
	text = transfer.str();
	return transfer.str();
}
template<class T>
std::string MatVis::toString(const T v, int w, int h)
{
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	return transfer.str();
}


//template<class T>
//std::string toStringCurrent(T v)
//{
//	transfer << v;
//}


std::string MatVis::VisIntArray(const int* v, int w, int h)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}


std::string MatVis::VisShortArray(const short* v, int w, int h)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();
}

std::string MatVis::VisUcharArray(const unsigned char* v, int w, int h)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			int temp = v[i*w + j];
			transfer << std::left << std::setw(MAXSETW) << temp;
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}




std::string MatVis::VisFloatArray(const float* v, int w, int h)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}


std::string MatVis::VisDoubleArray(const double* v, int w, int h)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}



std::string MatVis::VisEigen(const Eigen::Matrix3f& v)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>" << std::endl;


	transfer  << std::endl << v << std::endl;


	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}
std::string MatVis::VisEigen(const Eigen::Matrix4f& v)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>" << std::endl;
	
		transfer << "Mat " << std::endl << v << std::endl;
	
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}


std::string MatVis::VisEigen(const Eigen::Vector3f& v)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>" << std::endl;


		transfer <<  std::endl << v << std::endl;


	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}


std::string MatVis::VisEigen(const Eigen::VectorXf& v)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>" << std::endl;


		transfer  << std::endl << v << std::endl;


	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}




std::string MatVis::VisEigenVec2(const Eigen::Vector2f* v, int w, int h, int k)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j][k];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}


std::string MatVis::VisEigenVec3(const Eigen::Vector3f* v, int w, int h, int k)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j][k];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}




std::string MatVis::VisEigenVec4(const Eigen::Vector4f* v, int w, int h, int k)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>";
	transfer << std::setprecision(3);//<< std::fixed ;
	transfer << std::left << std::setw(MAXSETW) << "" << "      ";
	for (int j = 0; j < w; j++)
	{
		transfer << std::left << std::setw(MAXSETW) << j;
	}
	transfer << std::endl;
	transfer << std::left << std::setw(MAXSETW) << "";
	for (int j = 0; j < w; j++)
	{
		for (int i = 0; i < MAXSETW; i++)
			transfer << "-";
	}
	transfer << std::endl;
	transfer << std::endl;
	for (int i = 0; i < h; i++)
	{
		transfer << std::left << std::setw(MAXSETW) << i << "|     ";
		for (int j = 0; j < w; j++)
		{
			transfer << std::left << std::setw(MAXSETW) << v[i*w + j][k];
		}
		transfer << std::endl;
	}
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}


std::string MatVis::VisCVMat(const cv::Mat& v)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>" << std::endl;
	transfer << v;
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();


}




std::string MatVis::VisCVMatType(const cv::Mat& v)
{
	lock();
	text.clear();
	//transfer.flush();
	transfer.str("");
	transfer.clear();


	transfer << "<!DOCTYPE html>\n<html>\n<pre>" << std::endl;
	transfer << type2str(v.type());
	transfer << " </pre>\n</html>" << std::endl;
	text = transfer.str();
	unlock();
	return transfer.str();
}






MatVis* MatVis::Vis()//getInstance()
{
	static MatVis* instance;
	if (instance == NULL)
	{
		instance = new MatVis();
		int tempint[1] = { 0 };
		Vis()->VisIntArray(tempint, 1, 1);
		float tempfloat[1] = { 0 };
		Vis()->VisFloatArray(tempfloat, 1, 1);
		double tempdouble[1] = { 0 };
		Vis()->VisDoubleArray(tempdouble, 1, 1);
		uchar tempuchar[1] = { 0 };
		Vis()->VisUcharArray(tempuchar, 1, 1);
		
		Eigen::Matrix3f tempMatrix3f;
		Eigen::Matrix4f tempMatrix4f;
		Eigen::Vector3f tempVector3f;
		Eigen::VectorXf tempVectorXf;
		cv::Mat tempMat;
		Vis()->VisEigen(tempMatrix3f);
		Vis()->VisEigen(tempMatrix4f);
		Vis()->VisEigen(tempVector3f);
		Vis()->VisEigen(tempVectorXf);
		//Vis()->VisSE3(&tempSE3);
		Vis()->VisCVMat(tempMat);
		Vis()->VisCVMatType(tempMat);
		Eigen::Vector2f tempVec2[1];
		Vis()->VisEigenVec2(tempVec2, 1, 1, 0);
		Eigen::Vector3f tempVec3[1];
		Vis()->VisEigenVec3(tempVec3, 1, 1, 0);
		Eigen::Vector4f tempVec4[1];
		Vis()->VisEigenVec4(tempVec4, 1, 1, 0);
		Vis()->logToFile("MatVisLog.txt");
	}
	return instance;
}


void MatVis::logToFile(char* fileName)
{
	std::ofstream logFile(fileName);
	if (!logFile.is_open())
	{
		std::cout << "can't open Log file\n";
		return;
	}
	logFile << text;
	logFile.flush();
	logFile.close();
	std::cout << "Log file finish\n";
}


void MatVis::lock()
{
	//mutex.lock();
}
void MatVis::unlock()
{
	//mutex.unlock();
}










std::string type2str(int type) {
	std::string r;


	unsigned char depth = type & CV_MAT_DEPTH_MASK;
	unsigned char chans = 1 + (type >> CV_CN_SHIFT);


	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}


	r += "C";
	r += (chans + '0');


	return r;
}






















