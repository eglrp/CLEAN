#ifndef __TE_Pattern_BU__
#define __TE_Pattern_BU__

#include <iostream>
#include <map>
#include <set>
#include <string.h>

#include "pattern.h"


// struct arr_cmp {
//     bool operator () (const RoadSegment* a, const RoadSegment* b) const{
//         if (a[0] == b[0])
//             for (int i = 1; i < a[0]; i++)
//                 if (a[i] != b[i])
//                     return a[i] < b[i];
//         return a[0] < b[0];
//     }
// };

class PatternBU: public Pattern{
public:
    RoadSegment* roadSegments;
    vector<pair<int, int>>* positions;

    PatternBU(RoadSegment* rSs){
        this->roadSegments = new RoadSegment[rSs[0]];
        memcpy(this->roadSegments, rSs, sizeof(RoadSegment) * rSs[0]);
        positions = new vector<pair<int, int>>();
    }

    void show(){
        for(int i = 0; i < this->roadSegments[0]; i++)
            cout << this->roadSegments[i] << ' ';
        cout << "Key_Width: " << this->roadSegments[0]-1 <<" Pos_Number: "<< this->positions->size() 
                << " Gain: " << (this->roadSegments[0]-1)*this->positions->size() - 
                (this->roadSegments[0]-1) - this->positions->size()<< endl;
        // for(int i = 0; i < this->roadSegments[0]-1; i++)
        //     cout << (int)this->keyMarks[i] << ' ';
        cout << endl;
    } 

    ~PatternBU(){
        delete [] roadSegments;
        delete positions;

    }
};

class NeighborBU{
public:
    RoadSegment* fstRS;
    RoadSegment* sedRS;
    vector<pair<int, int>>* positions;

    NeighborBU(RoadSegment* lrs, RoadSegment* rrs){
        this->fstRS = new RoadSegment[lrs[0]];
        this->sedRS = new RoadSegment[rrs[0]];
        memcpy(this->fstRS, lrs, sizeof(RoadSegment) * lrs[0]);
        memcpy(this->sedRS, rrs, sizeof(RoadSegment) * rrs[0]);
        positions = new vector<pair<int, int>>();
    }
    
    ~NeighborBU(){
        delete [] fstRS;
        delete [] sedRS;
        delete positions;
    }
};

#endif