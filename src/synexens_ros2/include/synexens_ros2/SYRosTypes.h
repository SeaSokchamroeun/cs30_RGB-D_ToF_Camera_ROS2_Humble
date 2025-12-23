#ifndef SYRosTypes_H
#define SYRosTypes_H
#include "SYDataDefine.h"

struct SYCameraConfig
{
    Synexens::SYResolution CS30DepthResolution;
    Synexens::SYResolution CS30RGBResolution;
    Synexens::SYStreamType CS30StreamType;

    Synexens::SYResolution CS20DepthResolution;
    Synexens::SYStreamType CS20StreamType;

    Synexens::SYResolution CS20PDepthResolution;
    Synexens::SYStreamType CS20PStreamType;

    Synexens::SYResolution CS40DepthResolution;
    Synexens::SYStreamType CS40StreamType;

    Synexens::SYResolution CS40ProDepthResolution;
    Synexens::SYResolution CS40ProRGBResolution;
    Synexens::SYStreamType CS40ProStreamType;
};


#endif // SYRosTypes_H