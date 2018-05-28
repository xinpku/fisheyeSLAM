#include "debug_utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

cv::Mat result;
cv::Mat z_result;
double projectPoint_z;


bool PrintInfo::print_info = false;
int PrintInfo::print_depth = -1;
int PrintInfo::current_ID_global = -999;
int PrintInfo::need_print_ID_global = -999;
std::map<std::string,int> PrintInfo::print_flags;

DebugDrawer debugDrawer;



void DebugDrawer::update_frame(cv::Mat& im)
{
    original_frame = im.clone();
    if(original_frame.channels()<3) //this should be always true
        cvtColor(original_frame,original_frame,CV_GRAY2BGR);

    for(auto i = frame_map_.begin();i!=frame_map_.end();i++)
    {
        (*i).second = original_frame.clone();
    }
}
void DebugDrawer::clear_draw()
{
    for(auto i = frame_map_.begin();i!=frame_map_.end();i++)
    {
        (*i).second = original_frame.clone();
    }
}
void DebugDrawer::draw()
{
    for(auto i = frame_map_.begin();i!=frame_map_.end();i++)
    {
        cv::namedWindow( (*i).first,0);
        cv::imshow( (*i).first, (*i).second);
    }
}

void DebugDrawer::addFrame(std::string frame_name)
{
    frame_map_[frame_name] = original_frame.clone();
}

void DebugDrawer::active(std::string frame_name)
{
    if(frame_map_.find(frame_name)==frame_map_.end())
        addFrame(frame_name);

    active_name = frame_name;
}

cv::Mat& DebugDrawer::image()
{
    if(active_name=="")
        active(active_name);

    return frame_map_[active_name];
}

cv::Mat& DebugDrawer::image(std::string frame_name)
{
    active(frame_name);
    return image();
}
