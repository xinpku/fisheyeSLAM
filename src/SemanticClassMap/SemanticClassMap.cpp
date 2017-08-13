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
        case SemanticClass::nCar:
            carMap.insert(mp);
            break;
        case SemanticClass ::nPerson:
            personMap.insert(mp);
            break;
        case SemanticClass ::nObstacle:
            obstacleMap.insert(mp);
            break;
        case SemanticClass ::nParkinglots:
            parkinglotMap.insert(mp);
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
        case SemanticClass::nCar:
            carMap.erase(mp);
            break;
        case SemanticClass ::nPerson:
            personMap.erase(mp);
            break;
        case SemanticClass ::nObstacle:
            obstacleMap.erase(mp);
            break;
        case SemanticClass ::nParkinglots:
            parkinglotMap.erase(mp);
            break;
        default:
            break;
    }
}

ORB_SLAM2::SemanticMap::SemanticMap()
{
    objectMapAccessById.resize(6);

    objectMapAccessById[1] = &roadMap;
    objectMapAccessById[2] = &carMap;
    objectMapAccessById[3] = &personMap;
    objectMapAccessById[4] = &obstacleMap;
    objectMapAccessById[5] = &parkinglotMap;
}

void ORB_SLAM2::SemanticMap::clear()
{
    roadMap.clear();
    carMap.clear();
    personMap.clear();
    obstacleMap.clear();
    parkinglotMap.clear();
}
