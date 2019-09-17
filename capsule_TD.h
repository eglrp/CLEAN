#ifndef __TE_Capsule_TD__
#define __TE_Capsule_TD__

#include <iostream>
#include <fstream>
#include <vector>
#include <set>

#include "pattern_TD_generator.h"
#include "pattern_TD_heap.h"
#include "pattern.h"
#include "road_trajectory.h"
#include "timer.h"

using namespace std;

typedef set<RoadSegment*, arr_cmp> PatternSet;

class CapsuleTD: public Capsule{
public:
    TrajectoryDB* trajDB;
    TDGenerator* tdGenerator;
    TDHeap*  tdHeap;
    PatternSet updateSet;
    PatternSet resSet;
    pair<int, int>* posVec;
    int*  innerPos;
    int posVecNumber;
    ofstream* logout;
    // vector<pair<int, int>>* posVec = new vector<pair<int, int>>(); 
    // vector<RoadSegment*>* resPatterns = new vector<RoadSegment*>();
    map<int, set<pair<int, int>>> deletedSet;
    Timer* timer;

    CapsuleTD(TrajectoryDB* tdb, int k, ofstream* fout, Timer* tr){
        this->logout = fout;
        Capsule::K = k;
        this->trajDB = tdb;
        innerPos = new int[k+1]();
        this->timer = tr;
    }

    int judgeInterval(int len, int pos, int tid, int intervalLen){
        int ipos; 
        int relation = 0;
        for(int i = 0; i < posVecNumber; i++){
            ipos = posVec[i].second;
            if (ipos - intervalLen +1 > pos) break;
            if (ipos < pos - len + 1) continue;
            if (ipos <= pos && ipos-intervalLen + 1 >= pos - len + 1){
                relation = 2; 
                innerPos[++innerPos[0]] = ipos - (pos - len + 1);
                continue;
            } 
            return 1;
        }
        return relation;
    }

    void compressSingleTraj(int length){
        int tid = posVec[0].first;
        int tlen = this->trajDB->trajectoryList[tid]->length;
        int left = posVec[0].second - length + 1;
        int right = posVec[posVecNumber-1].second + Capsule::K - 1;
        if (right > tlen -1) right = tlen - 1;
        int relation = 0;
        int tmp_pos;
        for (int i = 0; i < posVecNumber; i++){
            tmp_pos = posVec[i].second;
            this->trajDB->trajectoryList[tid]->cprsMarks[tmp_pos] = length;
        }
        // cout << "left " << left << " right " << right << endl;
        // cout << "tid: " << tid << endl;
        RoadSegment* rs;
        for (int i = left; i <= right; i++){
            for (int j = Capsule::K; j >= 2; j--){
                pair<int, int> tmp_pair{j, i};
                if (i - j + 1 < 0) continue;
                if (deletedSet[tid].count(tmp_pair) > 0) {
                    // cout <<"del : " << j << ' ' << tid <<' '<<i<<endl;
                    continue;
                    } 
                rs = new RoadSegment[j+1]();
                this->trajDB->getSlice(tid, i - j + 1, i, rs);
                if (this->tdGenerator->patternTDMap.count(rs) <= 0){
                    deletedSet[tid].insert(tmp_pair); 
                    delete [] rs; continue;
                }
                relation = judgeInterval(j, i, tid, length);
                if (relation == 0) { delete [] rs; break;}
                // rs = new RoadSegment[j+1]();
                // this->trajDB->getSlice(tid, i - j + 1, i, rs);
                if (relation == 1) {
                    deletedSet[tid].insert(tmp_pair); 
                    this->tdGenerator->patternTDMap[rs]->removePos(tid, i);
                    // if (rs[0]==3 && rs[1]==1 && rs[2]==2){
                    //     cout << tid << ' ' << i << endl;
                    //     cout << this->tdGenerator->patternTDMap[rs]->posNumber <<endl;
                    // } 
                } else {
                    if (updateSet.count(rs) <= 0) {
                        // cout <<"contain : " << j << ' ' << tid <<' '<<i<<endl;
                        // cout << tid << ' ' << i << endl;
                        this->tdGenerator->patternTDMap[rs]->compressKey(length, this->innerPos);
                        // this->tdGenerator->patternTDMap[rs]->show();
                    }
                }
                if (this->updateSet.count(rs) == 0)
                    updateSet.insert(rs);
                else
                    delete [] rs;
                innerPos[0] = 0;  
            }
        }
    }

    void filterPosAndCompressTraj(PatternTD* pat){
        int length = pat->roadSegments[0]-1;
        // cout << "filterPos " << pat->positions.size() <<endl;
        pair<int, int> tmp;
        for (auto it = pat->positions->begin(); it != pat->positions->end(); it++){
            tmp = *it;
            if (posVecNumber == 0) posVec[posVecNumber++] = tmp;
            else if (tmp.first != posVec[posVecNumber-1].first){
                this->compressSingleTraj(length);
                posVecNumber = 0;
                posVec[posVecNumber++] = tmp;
            } else if (tmp.second - length >= posVec[posVecNumber-1].second){
                    posVec[posVecNumber++] = tmp;
            } 
        }
        if (posVecNumber > 0){
            this->compressSingleTraj(length);
            posVecNumber = 0;
        }
        for (auto it = updateSet.begin(); it != updateSet.end(); it++){
            this->tdGenerator->patternTDMap[*it]->clearDelPos();
            this->tdHeap->update(*it);
            delete [] *it;
        }
        updateSet.clear();
    }

    void topdownCompression(){
        this->posVec = new pair<int, int>[2000];
        this->posVecNumber = 0;
        HeapElementFloat top;
        Capsule::resPatterns->clear();
        while (tdHeap->size > 0) {
            top = tdHeap->heapPattern[1]; 
            tdHeap->pop();
            // for (int k = 0; k < top.key[0]; k++)
            //     cout << top.key[k] << ' ';
            // cout << " compressing..." <<endl;
            // cout << top.value << ' ' << this->tdGenerator->patternTDMap[top.key]->cprsKeyLen << endl;
            if (top.value <= 0) break;
            this->filterPosAndCompressTraj(this->tdGenerator->patternTDMap[top.key]);
            Capsule::resPatterns->push_back(top.key);
            Capsule::resPatternMarks->push_back(this->tdGenerator->patternTDMap[top.key]->keyMarks);
            // this->tdGenerator->patternTDMap[top.key]->show();
        }
        *logout << "result_patterns_number: " << resPatterns->size() <<endl;
    }

    void compressing(){
        *this->logout << "compressing_start(ms): "<< timer->getMiliSecond() << endl;
        this->tdGenerator = new TDGenerator(this->trajDB, Capsule::K, this->logout);
        this->tdGenerator->startupGenerator();
        this->segmentSetPtr = &this->tdGenerator->segmentSet;
        *this->logout << "patterns_generated(ms): "<< timer->getMiliSecond() << endl;
        this->tdHeap = new TDHeap(this->tdGenerator);
        this->tdHeap->init();
        *this->logout << "heap_inited(ms): " << timer->getMiliSecond() << endl;
        this->topdownCompression();
        // RoadSegment* trs = new RoadSegment[6]{6,1,2,3,4,5};
        // this->tdGenerator->patternTDMap[trs]->show();
        *this->logout << "topdown_compression(ms): " << timer->getMiliSecond() << endl;
    }

    ~CapsuleTD(){
        delete tdGenerator;
        delete tdHeap;
        delete [] posVec;
        delete [] innerPos;
        // delete logout;
    }
};

#endif
