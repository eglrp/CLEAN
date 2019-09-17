#include <iostream>

#include "config.h"
#include "experiment.h"
// #include "pre_fix_unit.h"
#include "pre_DP.h"
// #include "pre_DP_tripple.h"
// #include "pre_dynamic.h"
// #include "pre_dynamicExp.h"
// #include "pre_combine.h"
// #include "pre_A.h"

using namespace std;

void systemInitialize()
{
    //Config::TRAJECTORY_DB = "data/T-S-press_4cpp.txt";
    // Config::TRAJECTORY_DB = "extract_road_traj/trajs.txt";
    Config::TRAJECTORY_DB = "data/spatial.txt";
    Config::TRAJECTORY_T_DB = "data/temporal.txt";
    Config::TRAJECTORY_DB_TEXT = "data/enwik8";
    Config::TRAJECTORY_DB_TEST = "data/example.txt";
    Config::CAPSULE_TD_OUTPATH = "result_NY_td/";
    Config::CAPSULE_BU_OUTPATH = "result_capsule_bu/";
    //Config::CAPSULE_BU_PLUS_OUTPATH = "result_capsule_bu_plus/";
    //Config::CAPSULE_BU_PLUS_OUTPATH = "extract_road_traj/d1926/";
    Config::CAPSULE_BU_PLUS_OUTPATH = "result_NY_bu_plus/";
    Config::ROAD_NETWORK_NODE = "map/NY_Nodes.txt";
	Config::ROAD_NETWORK_EDGE = "map/NY_Edges.txt";
	Config::ROAD_NETWORK_GEOMETRY = "map/NY_EdgeGeometry.txt";
}

int main()
{
    systemInitialize();
    cout << "Start..." << endl;
    // Road network
	FileReader* nodeReader = new FileReader(Config::ROAD_NETWORK_NODE, false);
	FileReader* edgeReader = new FileReader(Config::ROAD_NETWORK_EDGE, false);
	FileReader* geoReader = new FileReader(Config::ROAD_NETWORK_GEOMETRY, false);
	
	Graph* graph = new Graph(nodeReader, edgeReader, geoReader);
	
	delete nodeReader;
	delete edgeReader;
	delete geoReader;

    Experiment* ex = new Experiment();
    // ex->baseBUPlusTest(30, 100);
    // ex->baseBUPlusTextTest(25, 300001);
    //ex->baseBUPlusTextTest(25, 5);
    //ex->decompression(25, 40000, 1);
    //ex->baseBUTest(25, 80000);
    //ex->baseTDTest(20, 100000);
    // Experiment 1 compression 
    for (int i = 2; i <= 30; i++){
     //   ex->baseTDTest(i, 40000);
    //    ex->baseBUTest(i, 40000);
    //    ex->baseBUPlusTest(i, 40000);
    }
    int trajNumbers[] = {10000, 20000, 40000, 60000, 80000};
    //int trajNumbers[] = {60000, 120000, 180000, 240000, 300000};
    for (int i = 0; i < 5; i++){
        ex->baseTDTest(25, trajNumbers[i], 100, 10);
    //    ex->baseBUTest(25, trajNumbers[i]);
        ex->baseBUPlusTest(25, trajNumbers[i], 100, 10);
    }
    //
    // Experiment 2 decompression
     for (int i = 2; i < 31; i++){
     //   ex->decompression(i, 40000, 0);
      //  ex->decompression(i, 40000, 1);
     }
     int trajNumbers2[] = {10000, 20000, 40000, 60000, 80000};
     for (int i = 0; i < 5; i++){
       // ex->decompression(25, trajNumbers2[i], 0);
       // ex->decompression(25, trajNumbers2[i], 1);
     }
        // ex->decompression(50, 40000, 1);
    //
    // Experiment 3 generate index
    //ex->indexGenerating();
    //
    // Experiment 4 query
    // ex->query(25, 80000, 0);
    // ex->query(25, 80000, 1);
    //
    // Experiment 5 generate NEAT trajectory 
    // ex->neatGenerating();
        // Get road network trajectory from GPS trajectory and map match result
    // cout << "Road network load complete." << endl;
	// PreProcessor::getInstance()->generateRoadNetTrajectory(
	// 	graph,
	// 	"data/trajs",
	// 	"data/spatial.txt",
	// 	"data/temporal.txt"
	// );
    return 1;
}
