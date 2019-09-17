#ifndef __TE_Pattern_BU_Heap__
#define __TE_Pattern_BU_Heap__

#include <iostream>
#include <string.h>
#include <limits.h>
#include <map>

#include "pattern_BU.h"
#include "heap.h"
#include "config.h"
#include "timer.h"


// struct HeapElement{
//     RoadSegment* key;
//     int value;
// };

class BUHeap{
public:
    vector<HeapElement> heapPattern; 
    // PatternGenerator* generatedPats;
    PositionMap positionMap;
    int size=0;
    BUGenerator* buGenerator;
    
    BUHeap(BUGenerator* ibu){
        buGenerator = ibu;
        init();
    }

    void init(){
        this->size = 0;
        HeapElement x;
        x.value = INT_MAX;
        heapPattern.push_back(x);
        for (auto it = buGenerator->patternBUMap.begin(); it != buGenerator->patternBUMap.end(); it++){
            this->insert(it->first, it->second->positions->size());
        }
    }

    void insert(RoadSegment* rs, int support){
        int i, fa;
        HeapElement ele = {rs, support};
        if (heapPattern.size() == this->size + 1) heapPattern.push_back(ele);
        for(i = ++this->size;ele.value > heapPattern[fa = i>>1].value; i = fa){
            heapPattern[i] = heapPattern[fa];
            positionMap[heapPattern[i].key] = i;
        }
        heapPattern[i] = ele;
        positionMap[rs] = i;
    }

    void bubbleDown(int i, HeapElement ele){
        int son;
        for (; (son = i << 1) <= this->size; i = son){
            if (son + 1 <= this->size && heapPattern[son + 1].value > heapPattern[son].value)
                son++;
            if (heapPattern[son].value > ele.value){
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

    void update(RoadSegment* rs, int support){
        if (positionMap.count(rs) == 0) {
            insert(rs, support);
        } else {
            if (support == heapPattern[positionMap[rs]].value) return;
            heapPattern[positionMap[rs]].value = support;
            bubbleDown(positionMap[rs], heapPattern[positionMap[rs]]);
        }
    }

    ~BUHeap(){
        // delete [] heapPattern;
    }
};

#endif