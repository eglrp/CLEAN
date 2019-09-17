#ifndef __TE_Capsule_output__
#define __TE_Capsule_output__

#include <map>
#include <iostream>
#include <string.h>
#include <vector>

#include "road_trajectory.h"
#include "pattern.h"
#include "config.h"
#include "capsule.h"
#include "huffman.h"

using namespace std;

typedef map<RoadSegment*, int, arr_cmp> PatternIDMap;

class BlockSet{
public:
    map<RoadSegment, int> baseSegMap;
    int patID = 0;
    PatternIDMap patIDMap;
    // vector<int>** cprsPatterns;
    vector<RoadSegment*>* patVec;
    Capsule* capsule;
    TrajectoryDB* trajDB;
    // vector<RoadSegment> baseSegVec;
    vector<vector<RoadSegment>*> blockVec;

    BlockSet(Capsule* cap, TrajectoryDB* trajDB, FileWriter* patWriter, ofstream* patout){
        this->capsule = cap;
        this->trajDB = trajDB;
        patVec = this->capsule->resPatterns;
        vector<int* >* patMarks = this->capsule->resPatternMarks;

        patWriter->writeInt(patVec->size());
        *patout << patVec->size() << endl;
        int baseNum = this->capsule->segmentSetPtr->size();
        *patout << baseNum << endl;
        patWriter->writeInt(baseNum);
        for (auto it = this->capsule->segmentSetPtr->begin(); it != this->capsule->segmentSetPtr->end(); it++ ){
            patWriter->writeInt(*it); 
            *patout << *it << ' '; 
            // this->trajDB->frequency[patID] = 0;
            baseSegMap[*it] = patID++;
        }
        *patout << endl; 

        vector<int> cprsPat;
        for (int i = 0; i < patVec->size(); i++){
            // this->trajDB->frequency[patID] = 0;
            patIDMap[patVec->at(i)] = patID++;
            bool greaterThanUShort = true;
            for (int j = patVec->at(i)[0]-1; j > 0; ){
                int len = patMarks->size() == 0 ? 0 : patMarks->at(i)[j-1];
                if (len != patVec->at(i)[0]-1 && len > 0) {
                    RoadSegment* smpat = new RoadSegment[len + 1]();
                    smpat[0] = len+1;
                    for (int k = 1; k < len+1; k++)
                        smpat[k] = patVec->at(i)[j-len+k];
                    int id = patIDMap[smpat];
                    cprsPat.push_back(id);
                    j -= len;
                } else {
                    cprsPat.push_back(baseSegMap[patVec->at(i)[j]]);
                    j--;
                }
            }
            reverse(cprsPat.begin(), cprsPat.end());
            int patLen = cprsPat.size();
            
            patWriter->writeInt(patLen);
            *patout << patID-1 << ' ';
            for (auto itr = cprsPat.begin(); itr != cprsPat.end(); itr++){
                *patout << *itr << ' ';
                patWriter->writeInt(*itr);
            }
            *patout << endl;
            cprsPat.clear();
        }
    }

    BlockSet(FileReader* blkFR){
        int patSize, baseNum, segment, blk;
        patSize = blkFR->nextInt();
        baseNum = blkFR->nextInt();
        for (int i = 0; i < baseNum; i++){
            segment = blkFR->nextInt();
            // baseSegMap[segment] = patID++;
            vector<RoadSegment>* segs = new vector<RoadSegment>(); 
            segs->push_back(segment);
            blockVec.push_back(segs);
            // baseSegVec.push_back(segment);
        }

        for (int i = 0; i < patSize; i++){
            int patLen = blkFR->nextInt();
            vector<RoadSegment>* segs = new vector<RoadSegment>(); 
            for (int j = 0; j < patLen; j++ ){
                blk = blkFR->nextInt();
                copy(blockVec[blk]->begin(),blockVec[blk]->end(), back_inserter(*segs)); 
            }
            blockVec.push_back(segs);
        }
    }

    ~BlockSet(){

    }
};

class CapsuleOutput{
public:
    BlockSet* blockSet;
    Capsule* capsule;
    TrajectoryDB* trajDB;


    CapsuleOutput(Capsule* cap, TrajectoryDB* trajDB){
        this->capsule = cap;
        this->trajDB = trajDB;
    }
    void capsuledTrajectoryStore(FileWriter* cprsTrajFW, ofstream* ptrout, FileWriter* hfmWriter, ofstream* oboout){

        cprsTrajFW->writeInt(this->trajDB->trajsNumber);
        *ptrout << this->trajDB->trajsNumber << endl;

        vector<int> cprsPat;
        for (int i = 0; i < this->trajDB->trajsNumber; i++){
//            bool greaterThanUShort = false;
            vector<int> obopats;
            for (int j = this->trajDB->trajectoryList[i]->length-1; j>=0; ){
                int len = this->trajDB->trajectoryList[i]->cprsMarks[j];
                if (len > 1){
                    RoadSegment* smpat = new RoadSegment[len + 1]();
                    smpat[0] = len+1;
                    for (int k = 1; k < len+1; k++)
                        smpat[k] = this->trajDB->getSegId(i, j-len+k);
                    int id = blockSet->patIDMap[smpat];
                    //if (id > USHRT_MAX) greaterThanUShort = true; 
                    // cprsPat.push_back(id);
                    this->trajDB->frequency[id]++;
                    this->trajDB->trajectoryList[i]->blockList.push_back(id);
                    // These codes are for pattern mining:
                    // begin
                    for (int k = 0; k < len; k++)
                        obopats.push_back(id);
                    // end
                    // if (i == 0) cout << id << " pat" <<endl;
                    j -= len;
                } else {
                    this->trajDB->frequency[blockSet->baseSegMap[this->trajDB->getSegId(i, j)]]++;
                    
                    obopats.push_back(this->blockSet->baseSegMap[this->trajDB->getSegId(i, j)]);
                    this->trajDB->trajectoryList[i]->blockList.push_back(
                        this->blockSet->baseSegMap[this->trajDB->getSegId(i, j)]);
                    // if (i == 0) cout << this->blockSet->baseSegMap[this->trajDB->getSegId(i, j)] << " " <<endl;
                    j--;
                }
            }
            reverse(this->trajDB->trajectoryList[i]->blockList.begin(),
                     this->trajDB->trajectoryList[i]->blockList.end());
            for (auto it = this->trajDB->trajectoryList[i]->blockList.begin();
                    it != this->trajDB->trajectoryList[i]->blockList.end(); it++){
                *ptrout << *it << ' ';
            }

            reverse(obopats.begin(), obopats.end());
            for (auto it = obopats.begin(); it != obopats.end(); it++){
                *oboout << *it << ' ';
            }
            *oboout << endl;
            *ptrout << endl;
        }
        HuffmanTree huffman(&(this->trajDB->frequency));
        huffman.store(hfmWriter);
        this->trajDB->compressedTrajStore(&huffman, cprsTrajFW);
    }

    void store(FileWriter* patternFW, FileWriter* cprsTrajFW, 
                ofstream* patTxt, ofstream* ptrTxt,
                FileWriter* hfmWriter, ofstream* oboTxt){
        this->blockSet = new BlockSet(this->capsule, this->trajDB, patternFW, patTxt);
        capsuledTrajectoryStore(cprsTrajFW, ptrTxt, hfmWriter, oboTxt);
    }
    ~CapsuleOutput(){

    }
};

#endif
