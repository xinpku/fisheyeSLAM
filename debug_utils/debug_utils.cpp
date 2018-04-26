#include "debug_utils.h"


cv::Mat result;
cv::Mat z_result;
double projectPoint_z;


bool PrintInfo::print_info = false;
int PrintInfo::print_depth = -1;
int PrintInfo::current_ID_global = -999;
int PrintInfo::need_print_ID_global = -999;
std::map<std::string,int> PrintInfo::print_flags;