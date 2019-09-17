#ifndef __TE_statistic__
#define __TE_statistic__

#include <iostream>
#include <cmath>
#include <map>
#include "road_trajectory.h"

using namespace std;

class Statistic{
public:
    map<int, int> item_freq;
    TrajectoryDB* trajDB; 
    int itemNumber = 0;
    float H = 0; 
    float Theory_space = 0;
    Statistic(TrajectoryDB* tdb){
        this->trajDB = tdb;
        for (int i = 0; i < tdb->trajsNumber; i++){
            for (int j =0; j < tdb->trajectoryList[i]->length; j++)
                ++item_freq[tdb->trajectoryList[i]->segmentList[j]];
            itemNumber += tdb->trajectoryList[i]->length;
        }   
        float tmp;
        for(auto pr = item_freq.begin();pr != item_freq.end(); ++pr){
            tmp = ((float)pr->second)/itemNumber;
            H += -tmp*log2(tmp);
        }
        Theory_space = H * itemNumber;
    }
};
#endif