#ifndef __TE_Heap__
#define __TE_Heap__

#include "pattern.h"

typedef map<RoadSegment*,int, arr_cmp> PositionMap;

struct HeapElement{
    RoadSegment* key;
    int value;
};

struct HeapElementFloat{
    RoadSegment* key;
    float value;
};

#endif