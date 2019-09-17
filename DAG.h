#ifndef __TE_DAG__
#define __TE_DAG__

#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <string.h>
#include <algorithm>

#include "node_pattern.h"
// #include "DAG_heap.h"

class DAG{
public:
    vector<NodePattern*> nodeVec;
    map<RoadSegment*, NodePattern*, arr_cmp> nodePatMap;
    
    DAG(PatternBUMap& pbm){
        for(auto it = pbm.begin(); it != pbm.end(); it++){
            NodePattern* np = new NodePattern(it->first, it->second->positions); 
            nodePushback(np);
        }
    }

    void nodePushback(NodePattern* np){
        np->id = nodeVec.size();
        nodeVec.push_back(np);
        nodePatMap[np->roadSegments] = np;
    }

    int getNodeId(RoadSegment* rs){
        //to do error check 
        return nodePatMap[rs]->id;
    }

    ~DAG(){

    }
};

#endif