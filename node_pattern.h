#ifndef __TE_Node_Pattern__
#define __TE_Node_Pattern__

#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <string.h>
#include <algorithm>

// #include "DAG_heap.h"
#include "pattern.h"
#include "pattern_BU_generator.h"

class NodePattern: public Pattern{
public:
    RoadSegment* roadSegments;
    vector<pair<int, int>>* positions;
    int id;
    int lsubPat = -1;
    int rsubPat = -1;
    set<int> supperPats;
    int cprsKeyLen;
    int* keyMarks = NULL;
    int cprsGain;

    NodePattern(RoadSegment* rSs, vector<pair<int, int>>* ps){
        this->roadSegments = new RoadSegment[rSs[0]];
        this->cprsKeyLen = rSs[0] - 1;
        this->keyMarks = new int[rSs[0]]();
        memcpy(this->roadSegments, rSs, sizeof(RoadSegment) * rSs[0]);
        positions = new vector<pair<int, int>>();
        *positions = *ps;
        calcCompressingGain();
    }

    void updateCprsKeyLen(){
        int delta = 0;
        for (int i = this->roadSegments[0]-2; i > 0;){
            if (this->keyMarks[i] > 1) {
                delta += this->keyMarks[i]-1;
                i -= this->keyMarks[i];
            } else {
                i--;
            }
        }
        this->cprsKeyLen = this->roadSegments[0] - 1 - delta;
        calcCompressingGain();
    }

    void calcCompressingGain(){
        this->cprsGain = this->cprsKeyLen * this->positions->size() 
                         - this->cprsKeyLen - this->positions->size(); 
    }

    void show(){
        for(int i = 0; i < this->roadSegments[0]; i++)
            cout << this->roadSegments[i] << ' ';
        cout << endl;
        cout << cprsGain;
        //cout << "Key_Width: " << this->roadSegments[0]-1 <<" Pos_Number: "<< this->positions->size() 
        //        << " Gain: " << (this->roadSegments[0]-1)*this->positions->size() - 
        //        (this->roadSegments[0]-1) - this->positions->size()<< endl;
        // for(int i = 0; i < this->roadSegments[0]-1; i++)
        //     cout << (int)this->keyMarks[i] << ' ';
        cout << endl;
    } 

    ~NodePattern(){
        delete [] roadSegments;
        delete positions;
    }
};


#endif