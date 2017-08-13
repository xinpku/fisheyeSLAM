//
// Created by xin on 17-7-22.
//

#ifndef ORB_SLAM2_ROADMAP_H
#define ORB_SLAM2_ROADMAP_H
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "MapPoint.h"
#include "SemanticClass.h"
#include <mutex>
#include "ObjectMap.h"

namespace ORB_SLAM2
{
    class ObjectMap;
    class SemanticMap
    {


    public:
        void addMapPoint(MapPoint* mp);
        void eraseMapPoint(MapPoint* mp);
        void clear();
        ObjectMap roadMap;
        ObjectMap carMap;
        ObjectMap personMap;
        ObjectMap obstacleMap;
        ObjectMap parkinglotMap;
    };


}



#endif //ORB_SLAM2_ROADMAP_H
