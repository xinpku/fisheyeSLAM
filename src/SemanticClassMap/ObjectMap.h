#pragma once
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "MapPoint.h"


namespace ORB_SLAM2
{
    class ObjectMap
    {
        std::set<MapPoint *> point_cloud;
        std::mutex mutex_point_cloud;
    public:
        void insert(MapPoint *mp)
        {
            lock();
            point_cloud.insert(mp);
            unlock();
        }

        void erase(MapPoint *mp)
        {
            lock();
            point_cloud.erase(mp);
            unlock();
        }

        void clear()
        {
            lock();
            point_cloud.clear();
            unlock();
        }

        vector<MapPoint*> getPointsVector()
        {
            unique_lock<mutex> lock(mutex_point_cloud);
            return vector<MapPoint*>(point_cloud.begin(),point_cloud.end());
        }

        void lock()
        {
            mutex_point_cloud.lock();
        }

        void unlock()
        {
            mutex_point_cloud.unlock();
        }

        set<MapPoint*>::iterator begin()
        {
            return point_cloud.begin();
        }

        set<MapPoint*>::iterator end()
        {
            return point_cloud.end();
        }

        int size()
        {
            return point_cloud.size();
        }

    };
}

