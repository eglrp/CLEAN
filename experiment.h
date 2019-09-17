#ifndef __TE_Experiment__
#define __TE_Experiment__

#include <iostream>
#include <map>
#include <string.h>
#include <fstream>

#include "config.h"
#include "file_processor.h"
#include "road_trajectory.h"
#include "statistic.h"
#include "capsule_TD.h"
#include "capsule_output.h"
#include "timer.h"
#include "capsule_BU.h"
#include "capsule_BU_plus.h"
#include "decompression.h"
#include "index_generator.h"
#include "road_network.h"
#include "auxiliary.h"
#include "query.h"
#include "neat_generator.h"
#include "temporal_cprs.h"

using namespace std;

class Experiment{
public:
    Graph* graph;

    Experiment(){
       // Road network
        FileReader* nodeReader = new FileReader(Config::ROAD_NETWORK_NODE, false);
        FileReader* edgeReader = new FileReader(Config::ROAD_NETWORK_EDGE, false);
        FileReader* geoReader = new FileReader(Config::ROAD_NETWORK_GEOMETRY, false);
        
        graph = new Graph(nodeReader, edgeReader, geoReader);
        
        delete nodeReader;
        delete edgeReader;
        delete geoReader; 
    }

    void fileOutput(ExprConfig& exConf, Capsule* capsule, TrajectoryDB* trajDB){
        FileWriter* patWriter = new FileWriter(exConf.getFilename("blk"), true); 
        FileWriter* trajWriter = new FileWriter(exConf.getFilename("ptr"), true); 
        FileWriter* hfmWriter = new FileWriter(exConf.getFilename("hfm"), true); 
        cout << exConf.getFilename("blk") << endl;
        CapsuleOutput* capsuleOutput = new CapsuleOutput(capsule, trajDB);
        ofstream* blktxt = new ofstream();
        blktxt->open(exConf.getFilename("blk.txt"));
        ofstream* ptrtxt = new ofstream();
        ptrtxt->open(exConf.getFilename("ptr.txt"));

        ofstream* obotxt = new ofstream();
        obotxt->open(exConf.getFilename("obo.txt"));
        capsuleOutput->store(patWriter, trajWriter, blktxt, ptrtxt, hfmWriter, obotxt);
        delete patWriter;
        delete trajWriter;
        delete hfmWriter;
        delete blktxt;
        delete ptrtxt;
    }

    void decompression(ExprConfig& exConf, ofstream* fout){
        FileReader* blkReader = new FileReader(exConf.getFilename("blk"), true);
        FileReader* hfmReader = new FileReader(exConf.getFilename("hfm"), true);
        FileReader* ptrReader = new FileReader(exConf.getFilename("ptr"), true);
        FileWriter* trajWriter = new FileWriter(exConf.getFilename("txt"), false);
        Decompression* decprs = new Decompression(blkReader, hfmReader, ptrReader, trajWriter, fout);
        delete blkReader;
        delete hfmReader;
        delete ptrReader;
        delete trajWriter;
    }

    void decompression(int para_k, int trajsNumber, int algo){
        ExprConfig exConfTD{Config::CAPSULE_TD_OUTPATH,
                          para_k,
                          trajsNumber,
                          "capsuleTD"};

        ExprConfig exConfBU{Config::CAPSULE_BU_PLUS_OUTPATH,
                          para_k,
                          trajsNumber,
                          "capsuleBUPlus"};

        ExprConfig confs[2] = {exConfTD, exConfBU};
        ofstream* fout = new ofstream();
        fout->open(confs[algo].getFilename("decprs.log"));
        decompression(confs[algo], fout);
        fout->close();

        delete fout;
    }

    void baseTDTest(int para_k, int trajNumber, int W, int Err){
        FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB, true);
        // FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB_TEST, false);
        TrajectoryDB* trajDB = new TrajectoryDB(trajsReader, trajNumber);
        ExprConfig exConf{Config::CAPSULE_TD_OUTPATH,
                          para_k,
                          trajDB->trajsNumber,
                          "capsuleTD", W, Err};
        TemporalCPRS* temporalCprs = new TemporalCPRS();
        temporalCprs->compress(Config::TRAJECTORY_DB, Config::TRAJECTORY_T_DB, exConf.getFilename("temporal"), trajNumber, W, Err);
        return;
        ofstream* fout = new ofstream();
        fout->open(exConf.getFilename("log"));
        Timer* timer = new Timer();
        timer->resetTimer();
        CapsuleTD* capsuleTD = new CapsuleTD(trajDB, exConf.K, fout, timer);
        capsuleTD->compressing();
        fileOutput(exConf, capsuleTD, trajDB);
        *fout << "TD_Compression_all_timecost(ms): "<< timer->getMiliSecond() << endl;
        fout->close();
        delete trajsReader;
        delete trajDB;
        delete capsuleTD;
        delete fout;
    }

    void baseBUTest(int para_k, int trajNumber, int W, int Err){
        FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB, true);
        //FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB_TEST, false);
        TrajectoryDB* trajDB = new TrajectoryDB(trajsReader, trajNumber);
        ExprConfig exConf{Config::CAPSULE_BU_OUTPATH,
                          para_k,
                          trajDB->trajsNumber,
                          "capsuleBU", W, Err};
        TemporalCPRS* temporalCprs = new TemporalCPRS();
        temporalCprs->compress(Config::TRAJECTORY_DB, Config::TRAJECTORY_T_DB, exConf.getFilename("temporal"), trajNumber, W, Err);
        //return;
        ofstream* fout = new ofstream();
        fout->open(exConf.getFilename("log"));
        Timer* timer = new Timer();
        timer->resetTimer();
        CapsuleBU* capsuleBU = new CapsuleBU(trajDB, exConf.K, fout, timer);
        capsuleBU->compressing();
        fileOutput(exConf, capsuleBU, trajDB);
        *fout << "BU_Compression_all_timecost(ms): "<< timer->getMiliSecond() << endl;
        fout->close();
        delete trajsReader;
        delete trajDB;
        delete capsuleBU;
        delete timer;
        delete fout;
    }

    void baseBUPlusTest(int para_k, int trajNumber, int W, int Err){
        FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB, true);
        // FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB_TEST, false);
        TrajectoryDB* trajDB = new TrajectoryDB(trajsReader, trajNumber);
        ExprConfig exConf{Config::CAPSULE_BU_PLUS_OUTPATH,
                          para_k,
                          trajDB->trajsNumber,
                          "capsuleBUPlus", W, Err};
        TemporalCPRS* temporalCprs = new TemporalCPRS();
        temporalCprs->compress(Config::TRAJECTORY_DB, Config::TRAJECTORY_T_DB, exConf.getFilename("temporal"), trajNumber, W, Err);
        return;
        cout << trajDB->trajsNumber << endl;
        ofstream* fout = new ofstream();
        fout->open(exConf.getFilename("log"));
        Timer* timer = new Timer();
        timer->resetTimer();
        CapsuleBUplus* capsuleBUplus = new CapsuleBUplus(trajDB, exConf.K, fout, timer);
        capsuleBUplus->compressing();
        fileOutput(exConf, capsuleBUplus, trajDB);
        *fout << "BU_Compression_all_timecost(ms): "<< timer->getMiliSecond() << endl;
        fout->close();
        delete trajsReader;
        delete trajDB;
        delete capsuleBUplus;
        delete timer;
        delete fout;
    }

    void baseBUPlusTextTest(int para_k, int trajNumber){
        // FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB_TEXT, false);
        FileReader* trajsReader = new FileReader(Config::TRAJECTORY_DB_TEST, false);
        TrajectoryDB* trajDB = new TrajectoryDB(trajsReader, trajNumber, true);
        //return;
        ExprConfig exConf{Config::CAPSULE_BU_PLUS_OUTPATH,
                          para_k,
                          trajDB->trajsNumber,
                          "capsuleBUPlus"};
        ofstream* fout = new ofstream();
        fout->open(exConf.getFilename("log"));
        Timer* timer = new Timer();
        timer->resetTimer();
        CapsuleBUplus* capsuleBUplus = new CapsuleBUplus(trajDB, exConf.K, fout, timer);
        capsuleBUplus->compressing();
        fileOutput(exConf, capsuleBUplus, trajDB);
        *fout << "BU_Compression_all_timecost(ms): "<< timer->getMiliSecond() << endl;
        fout->close();
        delete trajsReader;
        delete trajDB;
        delete capsuleBUplus;
        delete timer;
        delete fout;
    }


    void indexGenerating(){
       IndexGenerator* ig1 = new IndexGenerator("result_capsule_bu_plus");
       delete ig1;
       //ig1 = new IndexGenerator("result_capsule_bu_plus");
       //delete ig1;
       ig1 = new IndexGenerator("result_capsule_td");
       delete ig1;
    }

    void neatGenerating(){
        int trajs[5]{10000, 20000, 40000, 60000, 80000};
        //for (int i = 0; i < 5; i++){
        //    FileReader trajsReader(Config::TRAJECTORY_DB, false);
        //    string st1 = "NEAT_DATA/originTraj"+to_string(trajs[i])+".traj";
        //    string st2 = "NEAT_DATA/originTraj"+to_string(trajs[i])+".map";
        //    char* outTraj = strdup(st1.c_str());
        //    char* outMap = strdup(st2.c_str());
        //    FileWriter trajWriter(outTraj, false);
        //    FileWriter mapWriter(outMap, false);
        //    NEATGenerator::neatingOriginalTrajectory(this->graph, &trajsReader, trajs[i], &trajWriter, &mapWriter);
        //    delete outTraj;
        //    delete outMap;
        //}
        //
        //return ;
        cout << "original map and traj finish." << endl;
        char** paths = new char*[2]{Config::CAPSULE_TD_OUTPATH, Config::CAPSULE_BU_PLUS_OUTPATH};
        // char** paths = new char*[2]{Config::CAPSULE_TD_OUTPATH, Config::CAPSULE_BU_OUTPATH};
        char** algms = new char*[2]{"capsuleTD", "capsuleBUPlus"};
        int para_ks[2]{25,25};
        for (int i = 0; i < 2; i++){
            for (int j = 0; j < 5; j++){
                ExprConfig exConf{paths[i],
                            para_ks[i],
                            trajs[j],
                            algms[i]};
                FileReader blkReader(exConf.getFilename("blk"), true);
                FileReader hfmReader(exConf.getFilename("hfm"), true);
                FileReader ptrReader(exConf.getFilename("ptr"), true);
                string st1 = string("NEAT_DATA/")+exConf.getFilename("traj");
                string st2 = string("NEAT_DATA/")+exConf.getFilename("map");
                char* outTraj = strdup(st1.c_str());
                char* outMap = strdup(st2.c_str());
                FileWriter trajWriter(outTraj, false);
                FileWriter mapWriter(outMap, false);
                cout << "ready" << endl;
                cout << outTraj << endl;
                NEATGenerator::neatingCprsedTrajectory(this->graph, &hfmReader, &blkReader, &ptrReader, &trajWriter, &mapWriter);
                delete outTraj;
                delete outMap;
            }
        }

    }

    void query(int para_k, int trajN, int algo ){
        char* caps[2] = {"capsuleTD", "capsuleBU"};
        char* paths[2] = {Config::CAPSULE_TD_OUTPATH, Config::CAPSULE_BU_OUTPATH};
        ExprConfig exConf{paths[algo],
                          para_k,
                          trajN,
                          caps[algo]};

        FileReader* blkReader = new FileReader(exConf.getFilename("blk"), true);
        FileReader* hfmReader = new FileReader(exConf.getFilename("hfm"), true);
        FileReader* ptrReader = new FileReader(exConf.getFilename("ptr"), true);
        FileWriter* queryWriter = new FileWriter(exConf.getFilename("query"), false);
        Timer* timer = new Timer();
        timer->resetTimer();
        BlockSet* blockSet = new BlockSet(blkReader);
        Auxiliary* auxiliary = new Auxiliary(graph, blockSet); 
        HuffmanTree* huffman = new HuffmanTree(hfmReader);
        queryWriter->writeInt(timer->getMiliSecond());
        queryWriter->writeChar('\n');
        queryWriter->writeChar('\n');
        // cout<< timer->getMiliSecond() << endl;
        int trajNumber = ptrReader->nextInt();

        MBR** ranges = new MBR*[5]();
        ranges[0] = new MBR(31.2115964, 31.215677, 121.4658515, 121.4680203);
        ranges[1] = new MBR(31.2473755, 31.2488862, 121.4580497, 121.4583852);
        ranges[2] = new MBR(31.25208595, 31.2576649, 121.459088, 121.4592145);
        ranges[3] = new MBR(31.2520886, 31.2578317, 121.4589159,121.4590793);
        ranges[4] = new MBR(31.2401749, 31.24583742, 121.4576329, 121.4596481);
        for (int tid = 1; tid <= 50000; tid++){
            Binary* binary = new Binary(ptrReader);

            for (int rid = 0; rid < 5; rid++){
                bool result = Query::rangeOnCompressed(graph, huffman, blockSet, auxiliary, binary, ranges[rid]);
            }

            if (tid % 5000 == 0){ 
                queryWriter->writeInt(timer->getMiliSecond());
                queryWriter->writeChar('\n');
            }
        }
        delete queryWriter;
        delete blkReader;
        delete hfmReader;
        delete ptrReader;
        // cout<< timer->getMiliSecond() << endl;
    }

    ~Experiment(){

    }
};
#endif
