#ifndef __TE_Capsule_BU_plus__
#define __TE_Capsule_BU_plus__

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "road_trajectory.h"
#include "pattern_BU.h"
#include "DAG.h"
#include "DAG_heap.h"
#include "pattern_BU_generator.h"
#include "pattern_BU_heap.h"
#include "timer.h"

using namespace std;

typedef pair<int, int> IntPair;

class CapsuleBUplus : public Capsule{
public:
    TrajectoryDB* trajDB;
    BUGenerator* buGenerator;
    BUHeap* buHeap;
    ofstream* logout;
    Timer* timer;
    DAG* dag;
    DAGHeap* dagHeap;

    CapsuleBUplus(TrajectoryDB* tdb, int k, ofstream* fout, Timer* tr){
        this->trajDB = tdb;
        Capsule::K = k;
        this->logout = fout;
        this->timer = tr;
    }

    void addDAGNode(RoadSegment* lrs, RoadSegment* rrs, RoadSegment* rs) {
        NodePattern* np = new NodePattern(rs, buGenerator->patternBUMap[rs]->positions);
        dag->nodePushback(np);
        np->lsubPat = dag->getNodeId(lrs);
        np->rsubPat = dag->getNodeId(rrs);
        dag->nodeVec[np->lsubPat]->supperPats.insert(np->id);
        dag->nodeVec[np->rsubPat]->supperPats.insert(np->id);
    }

    void bottomUpCompression(){
        int dec = 0;
        while (buHeap->size > 0){
            HeapElement top = buHeap->heapPattern[1];
            bool isLeft;
            RoadSegment* rss = buGenerator->getMaxNeighbor(top.key, isLeft);
            if (rss == NULL) {
                buHeap->remove(top.key);
                continue;
            }
            RoadSegment* merge_rss;
            if (isLeft)
                merge_rss = buGenerator->merge(rss, top.key);
            else
                merge_rss = buGenerator->merge(top.key, rss);
            
            // Capsule::resPatterns->push_back(merge_rss);
            
            int* marks = new int[merge_rss[0]]();
            int width = merge_rss[0] - 1;
            int left_len = isLeft ? rss[0] - 1 : top.key[0] - 1;
            if (left_len > 1)
                marks[left_len - 1] = left_len;
            if (width - left_len > 1)
                marks[width - 1] = width - left_len; 
            // Capsule::resPatternMarks->push_back(marks);

            //if (buGenerator->patternBUMap[top.key]->positions->size() == 0) dec++;
            //if (buGenerator->patternBUMap[rss]->positions->size() == 0) dec++;

            //if (Capsule::resPatterns->size() - dec + 126 >= 63024){
            //    break;    
            //}
            if (isLeft) {
                addDAGNode(rss, top.key, merge_rss); 
            } else {
                addDAGNode(top.key, rss, merge_rss); 
            }
            buHeap->update(top.key, buGenerator->patternBUMap[top.key]->positions->size());
            buHeap->update(rss, buGenerator->patternBUMap[rss]->positions->size());
            buHeap->update(merge_rss, buGenerator->patternBUMap[merge_rss]->positions->size());
        }
    }

    int getCprsGain(int sup, int width){
        return sup*width - sup - width;
    }

    int descrptLen(RoadSegment* seq){
        return seq[0]-1 + buGenerator->patternBUMap[seq]->positions->size();
    }

    int getRelativePromotion(RoadSegment* seq, RoadSegment* left, RoadSegment* right){
        int option1 = descrptLen(seq) + descrptLen(left) + descrptLen(right);
        int option2 = descrptLen(left) + descrptLen(right) + 2 * buGenerator->patternBUMap[seq]->positions->size();
        return option2 - option1;
    }

    void encoding_D_with_Pattern(vector<pair<int, int>>* pos, int width){
        // Capsule::resPatterns->push_back(seq);
        for(auto it = pos->begin();
                 it != pos->end(); it++){
            trajDB->trajectoryList[it->first]->cprsMarks[it->second] = width;
        }
        // buGenerator->patternBUMap[seq]->show();
    }

    void resPatReorder(){
        int resPatSize = Capsule::resPatterns->size();
        IntPair* patList = new IntPair[resPatSize]; 
        for (int i = 0; i < resPatSize; i++){
            patList[i] = IntPair(Capsule::resPatterns->at(i)[0] - 1, i); 
        }
        sort(patList, patList + resPatSize);
        vector<RoadSegment*>* tmpResPat = new vector<RoadSegment*>();
        vector<int* >* tmpResPatMarks = new vector<int*>();
        for(int i = 0; i < resPatSize; i++){
            tmpResPat->push_back(Capsule::resPatterns->at(patList[i].second));
            tmpResPatMarks->push_back(Capsule::resPatternMarks->at(patList[i].second));
        }
        delete Capsule::resPatterns;
        delete Capsule::resPatternMarks;
        Capsule::resPatterns = tmpResPat;
        Capsule::resPatternMarks = tmpResPatMarks;
    }
    
    void greedySelection(){
        while (dagHeap->size > 0){
            HeapElement top = dagHeap->heapPattern[1];
            dagHeap->pop();
            // for (int k = 0; k < top.key[0]; k++)
                //  cout << top.key[k] << ' ';
            // cout << " compressing..." <<endl;
            // cout << top.value << ' ' << endl;
            if (top.value <= 0) break;
            encoding_D_with_Pattern(dag->nodePatMap[top.key]->positions, top.key[0] - 1);
            Capsule::resPatterns->push_back(top.key);
            Capsule::resPatternMarks->push_back(dag->nodePatMap[top.key]->keyMarks);
            dagHeap->updateSupperPats(top.key, top.key);
            dagHeap->updateSubPats(top.key, top.key, 0); 
        }
        resPatReorder();
    }

    void compressing(){
        *logout << "compressing_start(ms): "<< timer->getMiliSecond() << endl;
        buGenerator = new BUGenerator(this->trajDB, Capsule::K); 
        *logout << "Generated_BU_init_patterns(ms): "<< timer->getMiliSecond() << endl;
        this->segmentSetPtr = &this->buGenerator->segmentSet;
        buHeap = new BUHeap(buGenerator);
        dag = new DAG(this->buGenerator->patternBUMap);
        bottomUpCompression();

        *logout << "patterns_generated(ms): "<< timer->getMiliSecond() << endl;
        dagHeap = new DAGHeap(dag);
        greedySelection();
        *logout << "Compression_process(ms): "<< timer->getMiliSecond() << endl;
        // blockPacking();
        *logout << "Packing_blocks(ms): "<< timer->getMiliSecond() << endl;
    }

    ~CapsuleBUplus(){ 
        delete buGenerator;
        delete buHeap;
    }
};

#endif

