#ifndef __TE_Decompression__
#define __TE_Decompression__

#include <iostream>

#include "huffman.h"
#include "capsule_output.h"
#include "timer.h"

using namespace std;

class Decompression{
public:
    vector<vector<int>*> trajBlks;
    int trajNumber;
    Decompression(FileReader* hfmFR, FileReader* ptrFR){
        HuffmanTree* huffman = new HuffmanTree(hfmFR);
        trajNumber = ptrFR->nextInt();
        for (int tid = 0; tid < trajNumber; tid++){
            Binary* binary = new Binary(ptrFR);
            vector<int>* result = blkComplement(huffman, binary);
            trajBlks.push_back(result);
        }
    }

    Decompression(FileReader* blkFR, FileReader* hfmFR, FileReader* ptrFR, FileWriter* trajFW, ofstream* fout){
        Timer* timer = new Timer();   
        timer->resetTimer();
        HuffmanTree* huffman = new HuffmanTree(hfmFR);
        BlockSet* blockSet = new BlockSet(blkFR);
        *fout << "reconstruct_huffman_block(ms): " << timer->getMiliSecond()<<endl;
        trajNumber = ptrFR->nextInt();
        for (int tid = 0; tid < trajNumber; tid++){
            Binary* binary = new Binary(ptrFR);
		    HuffmanNode* node = huffman->getNode(huffman->root);
            vector<int>* result = blkComplement(huffman, binary);
            for (auto it = result->begin(); it != result->end(); it++){
                for (auto itt = blockSet->blockVec[*it]->begin(); itt != blockSet->blockVec[*it]->end(); itt++)
                    {trajFW->writeInt(*itt); trajFW->writeChar(' ');}
            }                    
            trajFW->writeChar('\n');
            delete binary;
            delete result; 
        }
        *fout << "decompression_all_timecost(ms): " << timer->getMiliSecond()<<endl;
        delete huffman;
        delete blockSet;

    }

    static vector<int>* blkComplement(HuffmanTree* huffman, Binary* binary){
        HuffmanNode* node = huffman->getNode(huffman->root);
        vector<int>* result = new vector<int>();
        for (int i = 0; i < binary->number; ++i) {
            if (binary->binary->at(i) == false) {
                node = huffman->getNode(node->leftSon);
            } else {
                node = huffman->getNode(node->rightSon);
            }
            if (node->leftSon == Config::NULL_POINTER && node->rightSon == Config::NULL_POINTER) {
                result->push_back(((HuffmanLeafNode*)node)->blockId);
                node = huffman->getNode(huffman->root);
            }
        }
        return result;
    }

    ~Decompression(){

    }
};

#endif