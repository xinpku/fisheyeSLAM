//
// Created by xin on 17-7-22.
//

#include "SemanticClassMap.h"

void ORB_SLAM2::SemanticMap::addMapPoint(ORB_SLAM2::MapPoint *mp)
{
    switch(mp->mSemanticClass)
    {
        case SemanticClass::nRoad:
            roadMap.insert(mp);
            break;
        default:
            break;
    }

}

void ORB_SLAM2::SemanticMap::eraseMapPoint(ORB_SLAM2::MapPoint* mp)
{
    switch(mp->mSemanticClass)
    {
        case SemanticClass::nRoad:
            roadMap.erase(mp);
            break;
        default:
            break;
    }
}
