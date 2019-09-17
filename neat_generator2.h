#ifndef __TE_NEAT_Generator__
#define __TE_NEAT_Generator__

#include <iostream>
#include <functional>
#include <vector>
#include <algorithm>
#include <map>
#include <set>

#include "file_processor.h"
#include "capsule_output.h"
#include "road_network.h"
#include "huffman.h"

using namespace std;

class NEATGenerator{
public:

    NEATGenerator(){
    }

    static void outputTrajLine(FileWriter* trajWriter,
             int tid, double lng, double lat, int sid){
        trajWriter->writeInt(tid);
        trajWriter->writeChar(',');
        trajWriter->writeDouble(lng);
        trajWriter->writeChar(',');
        trajWriter->writeDouble(lat);
        trajWriter->writeChar(',');
        trajWriter->writeInt(sid);
        trajWriter->writeChar('\n');
    }

    static void outputMapLine(FileWriter* mapWriter,
                int sid, double fstlng, double fstlat,
                double sedlng, double sedlat){
        mapWriter->writeInt(sid);
        mapWriter->writeChar(',');
        mapWriter->writeDouble(fstlng);
        mapWriter->writeChar(',');
        mapWriter->writeDouble(fstlat);
        mapWriter->writeChar(',');
        mapWriter->writeDouble(sedlng);
        mapWriter->writeChar(',');
        mapWriter->writeDouble(sedlat);
        mapWriter->writeChar('\n');
    
    }

    static void neatingOriginalTrajectory(Graph* graph, FileReader* trajReader, 
                    int trajNumber, FileWriter* trajWriter, FileWriter* mapWriter){
        TrajectoryDB trajDB(trajReader, trajNumber);
        for (int i = 0; i < trajDB.trajsNumber; i++){
            for (int j = 0; j < trajDB.trajectoryList[i]->length; j++){
                int sid = trajDB.getSegId(i, j);
                outputTrajLine(
                    trajWriter,
                    trajDB.trajectoryList[i]->id,
                    (graph->getEdge(sid)->startNode->location->y +
                    graph->getEdge(sid)->endNode->location->y) / 2,
                    (graph->getEdge(sid)->startNode->location->x +
                    graph->getEdge(sid)->endNode->location->x) / 2,
                    sid );
            } 
        }
        
        cout << "building map..." <<endl;
        
        for (int i = 0; i < graph->edgeList.size(); i++){
            int sid = graph->edgeList[i]->id;
            outputMapLine(
                mapWriter,
                sid,
                graph->getEdge(sid)->startNode->location->y,
                graph->getEdge(sid)->startNode->location->x,
                graph->getEdge(sid)->endNode->location->y,
                graph->getEdge(sid)->endNode->location->x
            );
            
        }
    }

    static void neatingCprsedTrajectory(Graph* graph, 
         FileReader* hfmReader, FileReader* blockReader, FileReader* ptrReader,
         FileWriter* trajWriter, FileWriter* mapWriter
         ){
        HuffmanTree* huffman = new HuffmanTree(hfmReader);
        BlockSet* blockSet = new BlockSet(blockReader);
        vector<vector<int>*> cprsTrajs;
        map<int, int> freq;
        int trajNumber = ptrReader->nextInt();
        for (int tid = 0; tid < trajNumber; tid++){
            Binary* binary = new Binary(ptrReader);
            vector<int>* result = Decompression::blkComplement(huffman, binary);
            cprsTrajs.push_back(result);
            for (auto it = result->begin(); it != result->end(); it++){
                freq[*it]++;
            }      
            delete binary;
        }
        IntPair* blockList = new IntPair[freq.size()];
        int i = 0;
        for (auto it = freq.begin(); it != freq.end(); ++it) {
            blockList[i++] = IntPair(it->second, it->first);
        }
        sort(blockList, blockList + freq.size());  
        reverse(blockList, blockList + freq.size());

        int max_blockId = blockSet->blockVec.size();
        cout << blockList[0].first <<endl;
        cout << blockList[1].first <<endl;
        set<int> segmentSet;
        set<int> outblocks;
        map<int, int> segMapping;
        for (int i = 0; i < freq.size(); i++){
            int canJoin = 0;
            int segSize = blockSet->blockVec[blockList[i].second]->size();
            vector<int> tmpVec;
            for (int j = 0; j < segSize; j++){
                if (j > 0){
                    int presid = blockSet->blockVec[blockList[i].second]->at(j-1);
                    int jsid = blockSet->blockVec[blockList[i].second]->at(j);
                    if (graph->getEdge(jsid)->startNode->id != graph->getEdge(presid)->endNode->id){
                        break;
                    }
                }
                if (segmentSet.count(blockSet->blockVec[blockList[i].second]->at(j)) > 0){
                    canJoin++;
                }
                tmpVec.push_back(blockSet->blockVec[blockList[i].second]->at(j));
            }
            //if (canJoin < 1 || (canJoin < 2 && tmpVec.size()>2) ){
            //if (true ){
            //if (tmpVec.size()-canJoin > 0){
            if (canJoin == 0){
                outblocks.insert(blockList[i].second);
                for (int j = 0; j < tmpVec.size(); j++){
                    if (segmentSet.count(tmpVec[j]) == 0)
                        segMapping[tmpVec[j]] = blockList[i].second;
                    segmentSet.insert(tmpVec[j]);
                }

            }
        }

        cout << trajNumber << ' ' << outblocks.size() << endl;
        for (int tid = 0; tid < trajNumber; tid++){
            for (auto it = cprsTrajs[tid]->begin(); it != cprsTrajs[tid]->end(); it++){
                if (outblocks.count(*it) > 0){
                    int sid = blockSet->blockVec[*it]->at(0);
                    outputTrajLine(
                        trajWriter,
                        tid,
                        graph->getEdge(sid)->startNode->location->y,
                        graph->getEdge(sid)->startNode->location->x,
                        *it);
                } else {
                    for (int k = 0; k < blockSet->blockVec[*it]->size(); k++){
                        int sid = blockSet->blockVec[*it]->at(k);
                        if (k > 0){
                            int presid = blockSet->blockVec[*it]->at(k-1);
                            int jsid = blockSet->blockVec[*it]->at(k);
                            if (graph->getEdge(jsid)->startNode->id != graph->getEdge(presid)->endNode->id){
                                break;
                            }
                        }       
                        //if (segMapping.count(sid) == 0) cout << "sid not in segMapping" << endl;
                        if (segMapping.count(sid) > 0){
                            int blk = segMapping[sid];
                            if (k > 0 && segMapping[blockSet->blockVec[*it]->at(k-1)] == segMapping[blockSet->blockVec[*it]->at(k)])
                                continue;
                            outputTrajLine(
                            trajWriter,
                            tid,
                            graph->getEdge(sid)->startNode->location->y,
                            graph->getEdge(sid)->startNode->location->x,
                            blk);
                        } else {
                            outputTrajLine(
                            trajWriter,
                            tid,
                            graph->getEdge(sid)->startNode->location->y,
                            graph->getEdge(sid)->startNode->location->x,
                            sid + max_blockId);

                            outblocks.insert(sid + max_blockId);
                        }
                    }
                }
            }
        }
        cout << "outblocks finish." << endl;
        int tmpsid;
        for (auto it = outblocks.begin(); it != outblocks.end(); it++){
            if (*it < max_blockId){
                int sid = blockSet->blockVec[*it]->at(0);
                int lastsid = blockSet->blockVec[*it]->at(0);
                
                for (int j = 1; j < blockSet->blockVec[*it]->size(); j++){
                    tmpsid = blockSet->blockVec[*it]->at(j);
                    if (tmpsid==11530) cout << "...here" << endl;
                    if (graph->getEdge(tmpsid)->startNode->id == graph->getEdge(lastsid)->endNode->id){
                        lastsid = tmpsid;
                    }
                }
                outputMapLine(
                    mapWriter,
                    *it,
                    graph->getEdge(sid)->startNode->location->y,
                    graph->getEdge(sid)->startNode->location->x,
                    graph->getEdge(lastsid)->endNode->location->y,
                    graph->getEdge(lastsid)->endNode->location->x
                );
            } else {
                int sid = *it - max_blockId;
                if (sid==11530) cout << *it <<" "<< freq.size() <<" ...there" << endl;
                outputMapLine(
                    mapWriter,
                    *it,
                    graph->getEdge(sid)->startNode->location->y,
                    graph->getEdge(sid)->startNode->location->x,
                    graph->getEdge(sid)->endNode->location->y,
                    graph->getEdge(sid)->endNode->location->x
                );
            }
        }
        delete huffman;
        delete blockSet;
        delete [] blockList;
        for (int i = 0; i < cprsTrajs.size(); i++)
            delete cprsTrajs[i];
        
    }

    ~NEATGenerator(){
    }
};

#endif
