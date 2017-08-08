//
// Created by xin on 17-7-22.
//

#ifndef ORB_SLAM2_ROADMAP_H
#define ORB_SLAM2_ROADMAP_H
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "MapPoint.h"
namespace ORB_SLAM2
{
    class SemanticMap
    {


    public:
        void addMapPoint(MapPoint* mp);
        void eraseMapPoint(MapPoint* mp);

        std::set<MapPoint*> roadMap;
    };

}



#endif //ORB_SLAM2_ROADMAP_H
