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
    
   // static void outputSegment(int tid, int sid, map<int, int>& segMapping, Graph* graph, FileWriter* trajWriter, set<int>& outblocks){
   //     if (segMapping.count(sid) > 0){
   //         int blk = segMapping[sid];
   //         outputTrajLine(
   //         trajWriter,
   //         tid,
   //         graph->getEdge(sid)->startNode->location->y,
   //         graph->getEdge(sid)->startNode->location->x,
   //         blk);

   //     } else {
   //         outputTrajLine(
   //         trajWriter,
   //         tid,
   //         graph->getEdge(sid)->startNode->location->y,
   //         graph->getEdge(sid)->startNode->location->x,
   //         sid + max_blockId);

   //         outblocks.insert(sid + max_blockId);
   //     }

   // }

    static bool isContiguous(int presid, int jsid, Graph* graph){
        return graph->getEdge(jsid)->startNode->id == graph->getEdge(presid)->endNode->id;
    }

    static void neatingCprsedTrajectory(Graph* graph, 
         FileReader* hfmReader, FileReader* blockReader, FileReader* ptrReader,
         FileWriter* trajWriter, FileWriter* mapWriter
         ){
        HuffmanTree* huffman = new HuffmanTree(hfmReader);
        BlockSet* blockSet = new BlockSet(blockReader);
        vector<vector<int>*> cprsTrajs;
        map<int, int> freq;
        map<vector<int>, int> contSegMap;
        vector<vector<int>> contVec;

        vector<vector<int>> block2CS;

        int contSegId = 0;

        for (int i = 0; i < blockSet->blockVec.size(); i++){
            vector<int> tmpVec; 
            tmpVec.push_back(blockSet->blockVec[i]->at(0));
            vector<int> block2cont;

            for (int j = 1; j < blockSet->blockVec[i]->size(); j++){
                int jsid = blockSet->blockVec[i]->at(j);
                if (isContiguous(tmpVec.back(), jsid, graph)){
                    tmpVec.push_back(jsid); 
                } else {
                    if (contSegMap.count(tmpVec) == 0) {
                        contSegMap[tmpVec] = contVec.size();
                        contVec.push_back(tmpVec);
                    }
                    block2cont.push_back(contSegMap[tmpVec]);
                    tmpVec.clear();
                    tmpVec.push_back(jsid);
                }
            }

            if (!tmpVec.empty()) {
                if (contSegMap.count(tmpVec) == 0) {
                    contSegMap[tmpVec] = contVec.size();
                    contVec.push_back(tmpVec);
                }
                block2cont.push_back(contSegMap[tmpVec]);
            }

            block2CS.push_back(block2cont);
        }

        int trajNumber = ptrReader->nextInt();
        for (int tid = 0; tid < trajNumber; tid++){
            Binary* binary = new Binary(ptrReader);
            vector<int>* result = Decompression::blkComplement(huffman, binary);
            vector<int>* contSegs = new vector<int>();
            for (auto it = result->begin(); it != result->end(); it++){
                for (auto cit = block2CS[*it].begin(); cit != block2CS[*it].end(); cit++){
                    freq[*cit]++;
                    contSegs->push_back(*cit);
                }
            }      
            cprsTrajs.push_back(contSegs);
            delete binary;
            delete result;
        }

        IntPair* contSegList = new IntPair[freq.size()];
        int i = 0;
        for (auto it = freq.begin(); it != freq.end(); ++it) {
            contSegList[i++] = IntPair(it->second, it->first);
        }
        sort(contSegList, contSegList + freq.size());  
        reverse(contSegList, contSegList + freq.size());

        int max_blockId = contVec.size();
        cout << contSegList[0].first <<endl;
        cout << contSegList[1].first <<endl;
        map<int, int> outblocks;
        vector<int> outblocksVec;
        map<int, int> segMapping;
        for (int i = 0; i < freq.size(); i++){
            int canJoin = 0;
            int segSize = contVec[contSegList[i].second].size();

            for (int j = 0; j < segSize; j++){
                if (segMapping.count(contVec[contSegList[i].second][j]) > 0){
                    canJoin++;
                    break;
                }
            }

            if (canJoin == 0){
                outblocksVec.push_back(contSegList[i].second);
                outblocks[contSegList[i].second] = outblocksVec.size() - 1;
                for (int j = 0; j < segSize; j++){
                    segMapping[contVec[contSegList[i].second][j]] = outblocksVec.size() - 1;
                }
            }
        }

        cout << trajNumber << ' ' << outblocks.size() << endl;
        int tid = 0;
        bool hasWrite = false;
        //for (int eid = 0; eid < graph->edgeList.size(); eid++){
        //    outblocksVec.push_back(eid + max_blockId);
        //    outblocks[eid + max_blockId] = outblocksVec.size() - 1;
        //}
        
        for (int tjs = 0; tjs < trajNumber; tjs++){
            for (auto it = cprsTrajs[tjs]->begin(); it != cprsTrajs[tjs]->end(); it++){
                if (outblocks.count(*it) > 0){
                    int sid = contVec[*it][0];
                    outputTrajLine(
                        trajWriter,
                        tid,
                        graph->getEdge(sid)->endNode->location->y,
                        graph->getEdge(sid)->endNode->location->x,
                        outblocks[*it]);
                        //*it);
                    hasWrite = true;
                } else {
                    for (int k = 0; k < contVec[*it].size(); k++){
                        int sid = contVec[*it][k];
                        if (segMapping.count(sid) > 0){
                            //if (hasWrite){ 
                            //    tid ++;
                            //    hasWrite = false;
                            //}
                            //continue;
                            outputTrajLine(
                            trajWriter,
                            tid,
                            graph->getEdge(sid)->startNode->location->y,
                            graph->getEdge(sid)->startNode->location->x,
                            segMapping[sid]);

                        } else {
                            if (outblocks.count(sid + max_blockId) == 0) {
                                outblocksVec.push_back(sid + max_blockId);
                                outblocks[sid + max_blockId] = outblocksVec.size() - 1;
                            }
                            outputTrajLine(
                            trajWriter,
                            tid,
                            graph->getEdge(sid)->startNode->location->y,
                            graph->getEdge(sid)->startNode->location->x,
                            outblocks[sid + max_blockId]);

                            hasWrite = true;
                            //outblocks.insert(sid + max_blockId);
                        }
                    }
                }
            }
            tid++;
        }
        cout << "outblocks finish." << endl;
        int tmpsid;
        int index = 0;
        for (auto it = outblocksVec.begin(); it != outblocksVec.end(); it++){
        //for (index = 0; index < max_blockId; ){
            if (*it < max_blockId){
            //if (true){
                int sid = contVec[*it][0];
                //int sid = contVec[index][0];
                int lastsid = contVec[*it].back();
                //int lastsid = contVec[index].back();
                
                outputMapLine(
                    mapWriter,
                    index,
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
                    index,
                    graph->getEdge(sid)->startNode->location->y,
                    graph->getEdge(sid)->startNode->location->x,
                    graph->getEdge(sid)->endNode->location->y,
                    graph->getEdge(sid)->endNode->location->x
                );
            }
            index++;
        }
        delete huffman;
        delete blockSet;
        delete [] contSegList;
        for (int i = 0; i < cprsTrajs.size(); i++)
            delete cprsTrajs[i];
        
    }

    ~NEATGenerator(){
    }
};

#endif
