#pragma once
enum SemanticClass
{
    nBackground = 0,
    nRoad = 1,
    nCar = 2,
    nPerson = 3,
    nObstacle = 4,
    nParkinglots = 5,
    nRejection = 6
};

//                                    back road  car   per   obst   park   rejection
const static int sematic_color_r[] = {-1,  -1,   255,   -1,   255,  255,   0};
const static int sematic_color_g[] = {-1,  -1,   255,   255,  -1,    -1,   0};
const static int sematic_color_b[] = {-1,  255,  -1,    -1,   -1,   255,   0};