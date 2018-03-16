#!/usr/bin/env bash
#./mono_video/mono_video Vocabulary/ORBvoc.txt calib_data/calib_fisheye.yaml /media/xin/data/data_outdoor/rear.avi calib_data/sw4066.txt None 200

#./mono_video/mono_video Vocabulary/ORBvoc.txt calib_data/calib_fisheye.yaml /media/xin/data/data_parking/front_19.avi calib_data/sw4066.txt /media/xin/data/data_parking/object_class 300
./mono_video/group_camera Vocabulary/ORBvoc.txt calib_data/calib_groupCamera.yaml /media/xin/data/data_parking/ calib_data/sw4066.txt None 300
