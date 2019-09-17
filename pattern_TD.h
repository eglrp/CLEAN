#ifndef __TE_Pattern_TD__
#define __TE_Pattern_TD__

#include <iostream>
#include <vector>
#include <string.h>
#include <algorithm>
#include <set>

#include "file_processor.h"
#include "pattern.h"
#include "capsule.h"

using namespace std;



class PatternTD: public Pattern{
public:
    RoadSegment* roadSegments;
    vector<pair<int, int>>* positions = new vector<pair<int, int> >(); 

    vector<pair<int, int>>* delPositions = new vector<pair<int, int>>();
    // set<pair <int,int> >* positions; 
    int cprsKeyLen;
    int cprsGain;
    int* keyMarks = NULL;
 
    PatternTD(RoadSegment* rSs){
        this->roadSegments = new RoadSegment[rSs[0]];
        this->keyMarks = new int[rSs[0]]();
        memcpy(this->roadSegments, rSs, sizeof(RoadSegment) * rSs[0]);
        this->cprsKeyLen = rSs[0]-1; 
    } 

    void calcCompressingGain(){
        this->cprsGain = this->cprsKeyLen * this->positions->size() - this->cprsKeyLen - this->positions->size(); 
    }

    void vec2Set(){
        // this->posMarks = new bool[this->positions->size()]();
        // positions = new set<pair<int, int> >(positionsVec->begin(), positionsVec->end());
        // this->posNumber = this->positions->size();
        calcCompressingGain(); 
    }

    void clearDelPos(){
        if (this->delPositions->size() == 0) return;         
        vector<pair<int, int>>* new_positions = new vector<pair<int, int>>(
            this->positions->size() - this->delPositions->size());
        set_difference(this->positions->begin(),this->positions->end(),
                       this->delPositions->begin(), this->delPositions->end(),
                       new_positions->begin());
        delete this->positions;
        this->positions = new_positions;
        this->delPositions->clear();

        calcCompressingGain();
    }

    void removePos(int tid, int pos){
        pair<int, int> one(tid, pos);
        this->delPositions->push_back(one);
        // if (this->positions->count(one) > 0) 
            // this->positions->erase(one);
        // calcCompressingGain();
    } 

    void compressKey(int len, int* innerPos){
        if (len == this->roadSegments[0]-1) return;
        for (int i = 1; i <= innerPos[0]; i++){
            int delta = len; 
            for (int j = innerPos[i]; j > innerPos[i]-len+2; j--){
                if (this->keyMarks[j] > 0) delta -= (this->keyMarks[j]-1);
            }
            this->keyMarks[innerPos[i]] = len;
            this->cprsKeyLen -= delta - 1;
        }
        calcCompressingGain();
    }

    void show(){
        for(int i = 0; i < this->roadSegments[0]; i++)
            cout << this->roadSegments[i] << ' ';
        cout << "Key_Width: " << this->cprsKeyLen <<" Pos_Number: "<< this->positions->size() 
                << " G: " << this->cprsKeyLen*this->positions->size()  - this->cprsKeyLen - this->positions->size()  << endl;
        for(int i = 0; i < this->roadSegments[0]-1; i++)
            cout << (int)this->keyMarks[i] << ' ';
        cout << endl;
    } 

    ~PatternTD() {
        delete [] roadSegments; 
        if (keyMarks != NULL)
            delete [] keyMarks;
    }
};

class PrefixPattern{
public:
    RoadSegment* roadSegments;
    vector<RoadSegment*> sufSegments; 
    PrefixPattern(RoadSegment* rSs){
        this->roadSegments = new RoadSegment[rSs[0]];
        memcpy(this->roadSegments, rSs, sizeof(RoadSegment) * rSs[0]);
    } 

    void show(){
        for(int i=0; i < roadSegments[0]; i++)
            cout << roadSegments[i] << ' ';
        cout<< sufSegments.size() <<endl;
    }

    ~PrefixPattern() {
       delete [] roadSegments; 
    }
};

#endif
