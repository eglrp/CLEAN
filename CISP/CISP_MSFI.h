#ifndef __TE_CISP_MSFI__
#define __TE_CISP_MSFI__

#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "road_trajectory.h"
#include "config.h"

using namespace std;

class Subsequence{
public:
    int tid;
    int start;
    int end;
    Subsequence(int tid, int st, int ed){
        this->tid = tid;
        this->start = st;
        this->end = ed;
    }
};


class MSFI{
public:
    map<int, int> item_freq;
    vector<int> items_ordered;
    map<int, vector<Subsequence*> > MSFIP;
    map<int, vector<Subsequence*> > MSFIS;
    TrajectoryDB* trajDB;
    MSFI(TrajectoryDB* tdb){
        this->trajDB = tdb;
        for (int i = 0; i < tdb->trajsNumber; i++)
            for (int j =0; j < tdb->trajectoryList[i]->length; j++)
                ++item_freq[tdb->trajectoryList[i]->segmentList[j]];

        vector<pair<int, int> > pairs;
        for (auto itr = item_freq.begin(); itr != item_freq.end(); ++itr)
            pairs.push_back(*itr);

        sort(pairs.begin(), pairs.end(), [=](std::pair<int, int>& a, std::pair<int, int>& b)
            {
                return a.second > b.second;
            }
        );

        for (int i = 0; i < pairs.size(); i++) {
            items_ordered.push_back(pairs[i].first);
        }
    }
    void buildMSFI_PS(){
        TrajectoryDB* tdb = this->trajDB;
        int max_freq = 0;
        //build MSFIP
        for (int i = 0; i < tdb->trajsNumber; i++){
            int st = 0;
            int ed = 0;
            for(;st < tdb->trajectoryList[i]->length;){
                max_freq = 0;
                ed = st;
                for(int j = st; j < tdb->trajectoryList[i]->length;j++){
                    if (item_freq[tdb->trajectoryList[i]->segmentList[j]] < Config::MIN_SUPPORT){
                        break;
                    }
                    if (item_freq[tdb->trajectoryList[i]->segmentList[j]] >= max_freq){
                        max_freq = item_freq[tdb->trajectoryList[i]->segmentList[j]];
                        ed = j;
                    }
                }
                if (max_freq)
                    this->MSFIP[tdb->trajectoryList[i]->segmentList[ed]].push_back(new Subsequence(i,st,ed));
                st = ed +1;
            }
        }

        //build MSFIS
        for (int i = 0; i < tdb->trajsNumber; i++){
            int st = 0;
            int ed = tdb->trajectoryList[i]->length;
            for(; ed > 0;){
                max_freq = 0;
                st = ed;
                for(int j = ed; j > 0;j--){
                    if (item_freq[tdb->trajectoryList[i]->segmentList[j]] < Config::MIN_SUPPORT){
                        break;
                    }
                    if (item_freq[tdb->trajectoryList[i]->segmentList[j]] >= max_freq){
                        max_freq = item_freq[tdb->trajectoryList[i]->segmentList[j]];
                        st = j;
                    }
                }
                if (max_freq)
                    this->MSFIS[tdb->trajectoryList[i]->segmentList[st]].push_back(new Subsequence(i,st,ed));
                ed = st - 1;
            }
        }
            

    }

    ~MSFI(){
    }
};


#endif