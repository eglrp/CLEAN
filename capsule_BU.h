#ifndef __TE_Capsule_BU__
#define __TE_Capsule_BU__

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "road_trajectory.h"
#include "pattern_BU.h"
#include "pattern_BU_generator.h"
#include "pattern_BU_heap.h"
#include "timer.h"

using namespace std;

class CapsuleBU : public Capsule{
public:
    TrajectoryDB* trajDB;
    BUGenerator* buGenerator;
    BUHeap* buHeap;
    ofstream* logout;
    Timer* timer;

    CapsuleBU(TrajectoryDB* tdb, int k, ofstream* fout, Timer* tr){
        this->trajDB = tdb;
        Capsule::K = k;
        this->logout = fout;
        this->timer = tr;
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

            Capsule::resPatterns->push_back(merge_rss);
            
            int* marks = new int[merge_rss[0]]();
            int width = merge_rss[0] - 1;
            int left_len = isLeft ? rss[0] - 1 : top.key[0] - 1;
            if (left_len > 1)
                marks[left_len - 1] = left_len;
            if (width - left_len > 1)
                marks[width - 1] = width - left_len; 
            Capsule::resPatternMarks->push_back(marks);

            if (buGenerator->patternBUMap[top.key]->positions->size() == 0) dec++;
            if (buGenerator->patternBUMap[rss]->positions->size() == 0) dec++;

            //if (Capsule::resPatterns->size() - dec + 126 >= 63024){
            //    break;    
            //}
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

    void blockMarking(RoadSegment* seq, int width){
        // Capsule::resPatterns->push_back(seq);
        if (width == 1) return;
        for(auto it = buGenerator->patternBUMap[seq]->positions->begin();
                 it != buGenerator->patternBUMap[seq]->positions->end(); it++){
            trajDB->trajectoryList[it->first]->cprsMarks[it->second] = width;
        }
        // buGenerator->patternBUMap[seq]->show();
    }

    void blockPacking(){
        vector<pair<int, RoadSegment*>> blocks;
        for (auto it = buGenerator->patternBUMap.begin(); it != buGenerator->patternBUMap.end(); it++){
            // blocks.push_back(pair<int, RoadSegment*>(it->first[0]-1, it->first));
            blockMarking(it->first, it->first[0]-1);
        }
        return;

        sort(blocks.begin(), blocks.end(),[](const std::pair<int,RoadSegment*> &left,
                                             const std::pair<int,RoadSegment*> &right) {
            return left.first > right.first;
        });
        int width, support, gain;
        for(auto it = blocks.begin(); it != blocks.end(); it++){
            width = it->first;
            support = buGenerator->patternBUMap[it->second]->positions->size();
            gain = getCprsGain(support, width);
            if (gain <= 0) continue;
            if (support > width)
                blockMarking(it->second, width);
            else {
                int posParent = -1;
                RoadSegment* left;
                RoadSegment* right;
                int rltGain = 0;
                int maxRltGain = 0; 
                for (int i = 0; i < buGenerator->parentMap[it->second].size(); i++){
                    left = buGenerator->parentMap[it->second][i].first;
                    right = buGenerator->parentMap[it->second][i].second; 
                    rltGain = getRelativePromotion(it->second, left, right);
                    if (rltGain > maxRltGain) {
                        maxRltGain = rltGain;
                        posParent = i;
                    } 
                }
                if (posParent != -1) {
                    left = buGenerator->parentMap[it->second][posParent].first;
                    right = buGenerator->parentMap[it->second][posParent].second;
                    buGenerator->unionPos(buGenerator->patternBUMap[left]->positions,
                                          buGenerator->patternBUMap[it->second]->positions, - right[0] + 1);
                    buGenerator->unionPos(buGenerator->patternBUMap[right]->positions,
                                          buGenerator->patternBUMap[it->second]->positions, 0);
                }
                else blockMarking(it->second, width);
            }
        }
    }
    
    void compressing(){
        *logout << "compressing_start(ms): "<< timer->getMiliSecond() << endl;
        buGenerator = new BUGenerator(this->trajDB, Capsule::K); 
        *logout << "Generated_BU_init_patterns(ms): "<< timer->getMiliSecond() << endl;
        this->segmentSetPtr = &this->buGenerator->segmentSet;
        buHeap = new BUHeap(buGenerator);
        bottomUpCompression();
        *logout << "Compression_process(ms): "<< timer->getMiliSecond() << endl;
        blockPacking();
        *logout << "Packing_blocks(ms): "<< timer->getMiliSecond() << endl;
    }

    ~CapsuleBU(){
        delete buGenerator;
        delete buHeap;
    }
};

#endif
