#ifndef __TE_DAG_Heap__
#define __TE_DAG_Heap__

#include <iostream>
#include <string.h>
#include <limits.h>
#include <map>

#include "DAG.h"
#include "pattern_BU.h"
#include "heap.h"
#include "config.h"
#include "timer.h"


// struct HeapElement{
//     RoadSegment* key;
//     int value;
// };

class DAGHeap{
public:
    HeapElement* heapPattern; 
    // PatternGenerator* generatedPats;
    PositionMap positionMap;
    int size=0;
    DAG* dag;
    DAGHeap(DAG* d){
        dag = d;
        heapPattern = new HeapElement[dag->nodeVec.size()+1];
        init();
    }

    void init(){
        this->size = 0;
        heapPattern[0].value = INT_MAX;
        for (auto it = dag->nodeVec.begin(); it != dag->nodeVec.end(); it++){
            this->insert((*it)->roadSegments,(*it)->cprsGain);
        }
    }

    bool isGreater(HeapElement a, HeapElement b){
        if (a.value == b.value) 
            return dag->nodePatMap[a.key]->cprsKeyLen < dag->nodePatMap[b.key]->cprsKeyLen;  
        return a.value > b.value;
    }

    void insert(RoadSegment* rs, int gain){
        int i, fa;
        HeapElement ele = {rs, gain};
        for(i = ++this->size; isGreater(ele, this->heapPattern[fa = i>>1]); i = fa){
            heapPattern[i] = heapPattern[fa];
            positionMap[heapPattern[i].key] = i;
        }
        heapPattern[i] = ele;
        positionMap[rs] = i;
    }

    void bubbleDown(int i, HeapElement ele){
        int son;
        for (; (son = i << 1) <= this->size; i = son){
            if (son + 1 <= this->size && isGreater(heapPattern[son + 1], heapPattern[son]))
                son++;
            if (isGreater(heapPattern[son], ele)){
                heapPattern[i] = heapPattern[son];
                positionMap[heapPattern[i].key] = i;
            } else 
                break;
        }
        heapPattern[i] = ele;
        positionMap[heapPattern[i].key] = i;
    } 

    void pop(){
        positionMap.erase(heapPattern[1].key);
        HeapElement p = heapPattern[this->size--];
        bubbleDown(1, p); 
    }

    void remove(RoadSegment* rs){
        if (positionMap.count(rs) == 0) return;
        HeapElement p = heapPattern[this->size--];
        bubbleDown(positionMap[rs], p);
        positionMap.erase(rs);
    }

    void update(RoadSegment* rs){
        if (positionMap.count(rs) <= 0) return;
        int new_G = dag->nodePatMap[rs]->cprsGain;
        //if (new_G == heapPattern[positionMap[rs]].value) return;
        heapPattern[positionMap[rs]].value = new_G;
        bubbleDown(positionMap[rs], heapPattern[positionMap[rs]]);
    }

    void differencePos(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2, int bias){
        // pos_set_difference(pos1, pos2, bias);
        // return;
        vector<pair<int, int>>* new_pos = new vector<pair<int, int>>(pos1->size());
        pair<int, int>* buffer;
        vector<pair<int, int>>::iterator it;
        if (bias == 0)
            it = set_difference(pos1->begin(), pos1->end(), pos2->begin(), pos2->end(), new_pos->begin());
        else{
            buffer = new pair<int, int>[pos2->size()];
            copy(pos2->begin(),pos2->end(), buffer);
            for (int i = 0; i < pos2->size(); i++)
                buffer[i].second += bias;
            it = set_difference(pos1->begin(), pos1->end(), buffer, buffer + pos2->size(), new_pos->begin());
        }
        new_pos->resize(it - new_pos->begin());  
        delete pos1;
        pos1 = new_pos;
    }

    void updateSupperPats(RoadSegment* rs , RoadSegment* key){
        NodePattern* np = dag->nodePatMap[rs];
        // np->show();
        for (auto it = np->supperPats.begin(); it != np->supperPats.end(); it++){
            NodePattern* supNp = dag->nodeVec[*it];
            // supNp->show();
            int key_len = key[0] - 1;
            for (int i = key[0]-1; i <= supNp->roadSegments[0]-1; i++){
                bool matched = true;
                for (int j = 1; j <= key_len; j++){
                    if (supNp->roadSegments[i - key_len + j] != key[j]) {
                        matched = false; 
                        break;
                    }
                }
                if (matched) supNp->keyMarks[i-1] = key_len;
            }
            supNp->updateCprsKeyLen();
            // supNp->show();
            this->update(supNp->roadSegments); 
            updateSupperPats(supNp->roadSegments, key);
        } 
    }

    void updateSubPats(RoadSegment* rs, RoadSegment* key, int bias){
        NodePattern* np = dag->nodePatMap[rs];
        NodePattern* keyNp = dag->nodePatMap[key];
        NodePattern* lNode = dag->nodeVec[np->lsubPat];
        NodePattern* rNode = dag->nodeVec[np->rsubPat];
        
        // lNode length is geater than 1.
        if (lNode->roadSegments[0] > 2) {
            differencePos(lNode->positions, keyNp->positions, bias - rNode->roadSegments[0] + 1);
            lNode->calcCompressingGain();
            this->update(lNode->roadSegments);
            updateSubPats(lNode->roadSegments, key, bias - rNode->roadSegments[0] + 1); 
        }

        if (rNode->roadSegments[0] > 2) {
            differencePos(rNode->positions, keyNp->positions, bias );
            rNode->calcCompressingGain();
            this->update(rNode->roadSegments);
            updateSubPats(rNode->roadSegments, key, bias); 
        }
    }


    ~DAGHeap(){
        // delete [] heapPattern;
    }
};

#endif