#ifndef __TE_Query__
#define __TE_Query__

#include <iostream>

#include "huffman.h"
#include "road_network.h"
#include "auxiliary.h"
#include "capsule_output.h"

using namespace std;
class Query{
public:

    static bool rangeOnCompressed(Graph* graph, HuffmanTree* huffman, BlockSet* blockSet,
                    Auxiliary* auxiliary, Binary* binary, MBR* range){
        vector<int>* blkList = Decompression::blkComplement(huffman, binary);
        for (int i = 0; i < blkList->size(); i++){
            if (auxiliary->fstMBR[blkList->at(i)]->intersect(range)){
                for(int j = 0; j < blockSet->blockVec[blkList->at(i)]->size(); j++){
                    int seg = blockSet->blockVec[blkList->at(i)]->at(j);
                    if (graph->getEdge(seg)->mbr->intersect(range)){
                        Node* n1 =graph->getEdge(blockSet->blockVec[blkList->at(i)]->at(j))->startNode;
                        Node* n2 =graph->getEdge(blockSet->blockVec[blkList->at(i)]->at(j))->endNode;
                        if (range->cross(n1->location, n2->location)) return true;
                    }
                }
            }
        }
        return false;
    }
};

#endif