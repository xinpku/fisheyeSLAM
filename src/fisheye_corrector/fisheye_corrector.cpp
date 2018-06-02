#include "fisheye_corrector.h"
#include <opencv2/features2d/features2d.hpp>




void FisheyeCorrector::readDistortionList(std::string file_name)
	{
		std::cout << "read distortion list" << std::endl;
		std::ifstream file(file_name);
		if (!file.is_open())
		{
			std::cout << "open file error";
			exit(-1);
		}

		distortion_list_.reserve(1035);
		float current_distortion;
		char skip;
		while (file >> current_distortion)
		{
			file >> skip; file >> skip;
			distortion_list_.push_back(current_distortion);
		}
		file.close();
	}

void FisheyeCorrector::generateMap()
{
	float angle_between_original_axis___ = (asin(sqrt(pow(sin(axis_vertical_radian_), 2) + pow(sin(axis_horizontal_radian_)*cos(axis_vertical_radian_), 2))));
	std::cout << "angle_between_original_axis___  " << angle_between_original_axis___ << std::endl;
	float angle_between_original_axis = radianToDegree(asin(sqrt(sin(axis_vertical_radian_)*sin(axis_vertical_radian_) / (1 - sin(axis_horizontal_radian_)*sin(axis_horizontal_radian_)*cos(axis_vertical_radian_)*cos(axis_vertical_radian_)))));
	std::cout << "angle_between_original_axis  " << angle_between_original_axis << std::endl;
	float  radius_in_fisheye = distortion_list_[angle_between_original_axis * 10] / pixelHeight_;
	f_image_ = angle_between_original_axis<0.01 ? f_camera_ : radius_in_fisheye / sin(degreeToRadian(angle_between_original_axis));
	//std::cout << "f_image_ " << f_image_ << std::endl;
	//std::cout << "horizontal_range_radian_ " << horizontal_range_radian_ << std::endl;
	//std::cout << "vertical_range_radian_  " << vertical_range_radian_ << std::endl;
	Width_ = tan(horizontal_range_radian_)*f_image_ * 2;
	Height_ = tan(vertical_range_radian_)*f_image_ * 2;
	std::cout << "width " << Width_ << " Height " << Height_ << std::endl;
	CenterX_ = (float)Width_ / 2.0f;
	CenterY_ = (float)Height_ / 2.0f;
	map_ = cv::Mat::ones(Height_, Width_, CV_32FC2)*(-1);
	
	float trans_y__ = sin(axis_vertical_radian_)*f_image_;
	float trans_x__ = cos(axis_vertical_radian_)*f_image_*sin(axis_horizontal_radian_);
	float trans_z__ = -f_camera_ + f_image_ * cos(degreeToRadian(angle_between_original_axis));
	//std::cout << "trans_x__  " << trans_x__ << std::endl;
	//std::cout << "trans_y__  " << trans_y__ << std::endl;
	float radian_between_original_axis = degreeToRadian(angle_between_original_axis);

	float trans_x = sin(radian_between_original_axis)*sin(axis_horizontal_radian_)*f_image_;
	float trans_y = sin(radian_between_original_axis)*cos(axis_horizontal_radian_)*f_image_;
	//std::cout << "trans_x  " << trans_x << std::endl;
	//std::cout << "trans_y  " << trans_y << std::endl;
	float trans_z = -f_camera_ + f_image_ * cos(radian_between_original_axis);
	new_camera_plane_center = Eigen::Vector3f(trans_x, trans_y, trans_z);
	//std::cout << "new_camera_plane_center  " << new_camera_plane_center.transpose() << std::endl;
	Eigen::Vector3f  original_camera_plane_center(0, 0, 0);
	camera_center = Eigen::Vector3f(0, 0, -f_camera_);

	Eigen::Quaternion<float> quaternion;
	original_axis = Eigen::Vector3f(original_camera_plane_center - camera_center).normalized();
	//std::cout << "original_axis " << original_axis.transpose() << std::endl;
	Eigen::Vector3f object_axis = (new_camera_plane_center - camera_center).normalized();
	//std::cout << "object_axis " << object_axis.transpose() << std::endl;
	quaternion.setFromTwoVectors(original_axis, object_axis);
	quaternion.normalize();
	
	Eigen::Quaternion<float> quaternion_axis(cos(axis_rotation_radian_/2), 0, 0, sin(axis_rotation_radian_/2));
	quaternion_axis.normalize();
	Eigen::Matrix3f rotation = quaternion_axis.toRotationMatrix()*quaternion.toRotationMatrix();
	std::cout<<"rotation*rotation"<<std::endl<<rotation*rotation<<std::endl;
	transform_camera_to_originalplane_ = Eigen::Matrix4f();
	transform_camera_to_originalplane_ <<
		rotation(0, 0), rotation(0, 1), rotation(0, 2), new_camera_plane_center(0),
		rotation(1, 0), rotation(1, 1), rotation(1, 2), new_camera_plane_center(1),
		rotation(2, 0), rotation(2, 1), rotation(2, 2), new_camera_plane_center(2),
		0,0,0,1;

	 T_camera_fisheye = Eigen::Matrix4f::Identity();
	 T_camera_fisheye.block(0,0,3,3) = rotation.transpose();

	//std::cout << "transform_camera_to_originalplane_" << std::endl << transform_camera_to_originalplane_ << std::endl;

	map_to_original_plane = cv::Mat::ones(Height_, Width_, CV_32FC2)*(-1);
	//std::ofstream log_file("points.txt");
	for (int h = 0; h < Height_; h++)
		for (int w = 0; w < Width_; w++)
		{
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);

		point_homo = transform_camera_to_originalplane_*point_homo;
		Eigen::Vector3f  point(point_homo(0), point_homo(1), point_homo(2));
		//std::cout << "point in center coordination " << point.transpose() << std::endl;
		//Eigen::Vector3f point_vector = (point - camera_center).normalized();

		float cos_value = original_axis.dot((point-camera_center).normalized());
		float degree = radianToDegree(acos(cos_value));
		if (degree > 100)
			continue;

		float x1 = point(0), x2 = 0, y1 = point(1), y2 = 0, z1 = point(2), z2 = -f_camera_;
		Eigen::Vector2f point_in_original_plane;
		//point_in_original_plane(0) = (x2* z1 - x1* z2) / (z1 - z2);
		//point_in_original_plane(1) = (y2* z1 - y1* z2) / (z1 - z2);
			point_in_original_plane(0) = x1*f_camera_/(z1+f_camera_);
			point_in_original_plane(1) = y1*f_camera_/(z1+f_camera_);
		//   	point_in_original_plane(0) = x1;
		//	point_in_original_plane(1) = y1;
		map_to_original_plane.at<cv::Vec2f>(h, w) = cv::Vec2f(point_in_original_plane(0), point_in_original_plane(1));

		//std::cout << "cosvalue  " << cos_value << "   " << original_axis.dot((Eigen::Vector3f(point_in_original_plane(0), point_in_original_plane(1), 0) - camera_center).normalized()) << std::endl ;

		float radius_in_project = sqrt(point_in_original_plane(0)*point_in_original_plane(0) + point_in_original_plane(1)*point_in_original_plane(1));
		if (radius_in_project == 0)
		{
			map_.at<cv::Vec2f>(h, w) = cv::Vec2f(CenterX_fisheye_, CenterY_fisheye_);
			continue;
		}
			

		int position_floor = floor(degree * 10);
		int position_ceil = ceil(degree * 10);
		float  radius_in_fisheye_floor = distortion_list_[position_floor];
		float  radius_in_fisheye_ceil = distortion_list_[position_ceil];
		float radius_in_fisheye;
		if (radius_in_fisheye_ceil == radius_in_fisheye_floor)
			radius_in_fisheye = radius_in_fisheye_ceil;
		else
			radius_in_fisheye = radius_in_fisheye_floor + (radius_in_fisheye_ceil - radius_in_fisheye_floor)*((degree * 10 - position_floor) / (position_ceil - position_floor));;
		radius_in_fisheye = radius_in_fisheye / pixelHeight_;

		float x = point_in_original_plane(0) *(radius_in_fisheye / radius_in_project);
		float y = point_in_original_plane(1)*(radius_in_fisheye / radius_in_project);

		//Add the map relationship of Point(h,w)
		//log_file << "pixel in fisheye coordinate  x " << x << "   y " << y << std::endl;
		map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, -y + CenterY_fisheye_);
		}
	//log_file.flush(); log_file.close();
	map_.copyTo(original_map_);
}



FisheyeCorrector::FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f, float vertical_range, float horizontal_range)
	:pixelHeight_(pixelHeight), f_camera_(f), horizontal_range_radian_(degreeToRadian(horizontal_range)), vertical_range_radian_(degreeToRadian(vertical_range))
	{
		size_scale_ = 1;
		CenterX_fisheye_ = input_width / 2.0f;
		CenterY_fisheye_ = input_height / 2.0f;
		readDistortionList(correction_table_file);
		std::cout << distortion_list_.size() << std::endl;
		if (f_camera_ < distortion_list_[1] / sin(degreeToRadian(0.1)))
		{
			std::cout << "focal length of camera is too small. Please check if it's correct." << std::endl;
			exit(-1);
		}
		

		axis_vertical_radian_ = 0;
		axis_horizontal_radian_ = 0;
		axis_rotation_radian_ = 0;


		
		clip_region_ = cv::Rect(0, 0, 0, 0);
		map_need_update = true;
		new_camera_plane_center = Eigen::Vector3f(0,0,0);
		camera_center = Eigen::Vector3f(0,0,0);
		original_axis = Eigen::Vector3f(0,0,0);

		transform_camera_to_originalplane_ = Eigen::Matrix4f();
        T_camera_fisheye = Eigen::Matrix4f::Identity();

	}





template<>
void FisheyeCorrector::mapToOriginalImage<cv::KeyPoint>(const std::vector<cv::KeyPoint>& points, std::vector<cv::KeyPoint>& points_in_fisheye)
{
	std::vector<cv::KeyPoint> points_in_fisheye_temp;
	points_in_fisheye_temp.resize(points.size());
	int width = map_.cols;
	int height = map_.rows;
	for (int i = 0; i < points.size(); i++)
	{
		float h = points[i].pt.y;
		float w = points[i].pt.x;
		//Transform the points in the corrected image to it's correct position

		if (h >= height || w >= width||h<0||w<0)
		{
			points_in_fisheye_temp[i] = points[i];
			points_in_fisheye_temp[i].pt.x = -1;
			points_in_fisheye_temp[i].pt.y = -1;
			continue;
		}

		float x = map_.at<cv::Vec2f>(h, w)(0);
		float y = map_.at<cv::Vec2f>(h, w)(1);
		//std::cout << "x " << x << "   y " << y << std::endl;
		//Add the map relationship of Point(h,w)
		points_in_fisheye_temp[i] = points[i];
		points_in_fisheye_temp[i].pt.x = x;
		points_in_fisheye_temp[i].pt.y = y;
	}
	points_in_fisheye.clear();
	points_in_fisheye.insert(points_in_fisheye.end(), points_in_fisheye_temp.begin(), points_in_fisheye_temp.end());
}


template<>
void FisheyeCorrector::mapFromCorrectedImageToCenterImagePlane<cv::KeyPoint>(const std::vector<cv::KeyPoint>& points, std::vector<cv::KeyPoint>& points_in_pinhole, float cx, float cy, float f_center_image)
{
	std::vector<cv::KeyPoint> points_in_pinhole_temp;
	points_in_pinhole_temp.resize(points.size());
	double ratio = f_center_image / f_camera_;
	int width = map_to_original_plane_clip.cols;
	int height = map_to_original_plane_clip.rows;
	//std::cout << "f_center_image " << f_center_image << " f_camera " << f_camera_ << std::endl;
	//std::cout << "ratio " << ratio << std::endl;
	for (int i = 0; i < points.size(); i++)
	{
		float h = points[i].pt.y;
		float w = points[i].pt.x;
		if (h >= height || w >= width || h<0 || w<0)
		{
			points_in_pinhole_temp[i] = points[i];
			points_in_pinhole_temp[i].pt.x = -1;
			points_in_pinhole_temp[i].pt.y = -1;
			continue;
		}
		//Transform the points in the corrected image to it's correct position
		float x = map_to_original_plane_clip.at<cv::Vec2f>(h, w)(0);
		float y = map_to_original_plane_clip.at<cv::Vec2f>(h, w)(1);
		//std::cout << "xo " << x << "   y " << y << std::endl;
		//Add the map relationship of Point(h,w)
		points_in_pinhole_temp[i] = points[i];
		points_in_pinhole_temp[i].pt.x = x*ratio + cx;
		points_in_pinhole_temp[i].pt.y = -y*ratio + cy;
	}
	points_in_pinhole.clear();
	points_in_pinhole_temp.swap(points_in_pinhole);
}