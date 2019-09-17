#ifndef __TE_config__
#define __TE_config__

using namespace std;

class Config{
public:
    static char* TRAJECTORY_DB;
    static char* TRAJECTORY_T_DB;
    static char* TRAJECTORY_DB_TEST;
    static char* TRAJECTORY_DB_TEXT;
    static char* CAPSULE_TD_OUTPATH;
    static char* CAPSULE_BU_OUTPATH; 
    static char* CAPSULE_BU_PLUS_OUTPATH; 
    static const int MIN_SUPPORT;
    static const int HUGE_NUMBER;
    static const int NULL_POINTER;
    static char* ROAD_NETWORK_NODE;			// road network node file
	static char* ROAD_NETWORK_EDGE;			// road network edge file
	static char* ROAD_NETWORK_GEOMETRY;		// road network geometry file
};

char* Config::TRAJECTORY_DB = NULL;
char* Config::TRAJECTORY_T_DB = NULL;
char* Config::TRAJECTORY_DB_TEST = NULL;
char* Config::TRAJECTORY_DB_TEXT = NULL;
char* Config::CAPSULE_TD_OUTPATH = NULL;
char* Config::CAPSULE_BU_OUTPATH = NULL;
char* Config::CAPSULE_BU_PLUS_OUTPATH = NULL;

char* Config::ROAD_NETWORK_NODE = NULL;
char* Config::ROAD_NETWORK_EDGE = NULL;
char* Config::ROAD_NETWORK_GEOMETRY = NULL;
const int Config::MIN_SUPPORT = 2;
const int Config::HUGE_NUMBER = 99999999;
const int Config::NULL_POINTER = -1;

#endif // __TE_config__

