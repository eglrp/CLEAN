#ifndef __TE_Index_Generator__
#define __TE_Index_Generator__

#include <iostream>
#include <string.h>

#include "file_processor.h"
#include "capsule_output.h"

using namespace std;

class IndexGenerator{
public:
    map<int, set<int> >* blk2traj;

    IndexGenerator(char* path){
        vector<char*>* files = FileTool::getInstance()->getExprFiles(path);
        for (auto it = files->begin(); it != files->end(); it++)
            generating(*it);
    }

    void generating(char* file){
        ExprResult expr{file};
        FileReader* hfmReader = new FileReader(expr.getFilename("hfm"), true);
        FileReader* ptrReader = new FileReader(expr.getFilename("ptr"), true);
        FileReader* blkReader = new FileReader(expr.getFilename("blk"), true);
        FileWriter* sbWriter = new FileWriter(expr.getFilename("segblk"), false);
        FileWriter* btWriter = new FileWriter(expr.getFilename("blktraj"), false);

        indexBlk2Traj(hfmReader, ptrReader, btWriter);
        indexSeg2Blk(blkReader, sbWriter);

        delete hfmReader;
        delete ptrReader;
        delete blkReader;
        delete sbWriter;
        delete btWriter;
    }

    void indexSeg2Blk(FileReader* blkReader, FileWriter* segBlkWriter){
        BlockSet* blockSet = new BlockSet(blkReader);
        map<int, set<int>> seg2blk;
        for (int i = 0; i < blockSet->blockVec.size(); i++){
            if (blk2traj->count(i) == 0) continue;
            for (auto it = blockSet->blockVec[i]->begin();
                    it != blockSet->blockVec[i]->end(); it++){
               seg2blk[*it].insert(i); 
            }
        }
        int sumSize =0;        
        for (auto it = seg2blk.begin(); it != seg2blk.end(); it++)
            sumSize += it->second.size();
        segBlkWriter->writeInt(sumSize);
        segBlkWriter->writeChar('\n');
        for (auto it = seg2blk.begin(); it != seg2blk.end(); it++){
            segBlkWriter->writeInt(it->first);
            segBlkWriter->writeChar(' ');            
            for (auto ibk = it->second.begin(); ibk != it->second.end(); ibk++)
                {segBlkWriter->writeInt(*ibk); segBlkWriter->writeChar(' ');}
            segBlkWriter->writeChar('\n');            
        }
    }

    void indexBlk2Traj(FileReader* hfmReader, FileReader* ptrReader, 
                        FileWriter* blkTrajWriter){
        Decompression decprsTraj(hfmReader, ptrReader);
        blk2traj = new map<int, set<int>>();
        int sumSize = 0;
        for (int i = 0; i < decprsTraj.trajBlks.size(); i++){
            for (auto it = decprsTraj.trajBlks[i]->begin();
                        it != decprsTraj.trajBlks[i]->end(); it++){
                (*blk2traj)[*it].insert(i);
            }
        }
        for (auto it = blk2traj->begin(); it !=blk2traj->end(); it++)
            sumSize += it->second.size();
        blkTrajWriter->writeInt(sumSize);
        blkTrajWriter->writeChar('\n');

        for (auto it = blk2traj->begin(); it != blk2traj->end(); it++){
            blkTrajWriter->writeInt(it->first);
            blkTrajWriter->writeChar(' ');            
            for (auto ibk = it->second.begin(); ibk != it->second.end(); ibk++)
                {blkTrajWriter->writeInt(*ibk); blkTrajWriter->writeChar(' ');}
            blkTrajWriter->writeChar('\n');            
        }
    }

    ~IndexGenerator(){

    }
};

#endif
