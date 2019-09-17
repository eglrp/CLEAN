#ifndef __TE_Capsule__
#define __TE_Capsule__

#include <iostream>

typedef set<RoadSegment> SegmentSet;

using namespace std;

class Capsule{
public:
    vector<RoadSegment*>* resPatterns = new vector<RoadSegment*>();
    vector<int* >* resPatternMarks = new vector<int*>();
    SegmentSet* segmentSetPtr;
    int K;
    Capsule(){
    }
    ~Capsule(){
    }
};

#endif