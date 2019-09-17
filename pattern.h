#ifndef __TE_Pattern__
#define __TE_Pattern__

#include <iostream>
#include <set>

using namespace std;

typedef int RoadSegment;


struct arr_cmp {
    bool operator () (const RoadSegment* a, const RoadSegment* b) const{
        if (a[0] == b[0])
            for (int i = 1; i < a[0]; i++)
                if (a[i] != b[i])
                    return a[i] < b[i];
        return a[0] < b[0];
    }
};

class Pattern{
public:
    // RoadSegment* roadSegments = NULL;
    // int* keyMarks = NULL;

    Pattern(){

    }

    ~Pattern(){

    }
};

#endif