#ifndef __TE_Pattern_TD_Heap__
#define __TE_Pattern_TD_Heap__

#include <iostream>
#include <string.h>
#include <limits.h>
#include <map>

#include "pattern_TD_generator.h"
#include "config.h"
#include "heap.h"
#include "timer.h"


class TDHeap{
public:
    HeapElementFloat* heapPattern; 
    TDGenerator* tdGenerator;
    PositionMap positionMap;
    int size;
    TDHeap(TDGenerator* tdGen){
        tdGenerator = tdGen;
        heapPattern  = new HeapElementFloat[tdGen->patternTDMap.size()+1];
    }

    void init(){
        this->size = 0;
        heapPattern[0].value = INT_MAX;
        for (auto it = tdGenerator->patternTDMap.begin(); it != tdGenerator->patternTDMap.end(); it++){
            this->insert(it->first);
        }
    }

    bool isGreater(HeapElementFloat a, HeapElementFloat b){
        if (a.value == b.value) 
            return tdGenerator->patternTDMap[a.key]->cprsKeyLen < tdGenerator->patternTDMap[b.key]->cprsKeyLen;  
        return a.value > b.value;
    }

    void insert(RoadSegment* rs){
        int i, fa;
        HeapElementFloat ele = {rs, (float)tdGenerator->patternTDMap[rs]->cprsGain};
        for(i = ++this->size; isGreater(ele, heapPattern[fa = i>>1]); i = fa){
            heapPattern[i] = heapPattern[fa];
            positionMap[heapPattern[i].key] = i;
        }
        heapPattern[i] = ele;
        positionMap[rs] = i;
    }

    void bubbleDown(int i, HeapElementFloat ele){
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
        int i, son;
        positionMap.erase(heapPattern[1].key);
        HeapElementFloat p = heapPattern[this->size--];
        bubbleDown(1, p); 
    }

    void update(RoadSegment* rs){
        if (positionMap.count(rs) <= 0) return;
        float new_G = tdGenerator->patternTDMap[rs]->cprsGain;
        //if (new_G == heapPattern[positionMap[rs]].value) return;
        heapPattern[positionMap[rs]].value = new_G;
        bubbleDown(positionMap[rs], heapPattern[positionMap[rs]]);
    }

    ~TDHeap(){
        delete [] heapPattern;
    }
};

#endif
