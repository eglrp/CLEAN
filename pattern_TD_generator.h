#ifndef __TE_Pattern_TD_Generator__
#define __TE_Pattern_TD_Generator__

#include <iostream>
#include <map>
#include <string.h>
#include <vector>
#include <limits.h>
#include <fstream>

#include "pattern_TD.h"
#include "road_trajectory.h"
#include "config.h"
#include "timer.h"

using namespace std;

typedef map<RoadSegment*, PrefixPattern*, arr_cmp> PrefixPatternMap;
typedef map<RoadSegment*, PatternTD*, arr_cmp> PatternTDMap;

class TDGenerator{
public:
    int K;
    SegmentSet segmentSet;
    TrajectoryDB* trajDB;
    PatternTDMap patternTDMap;    
    PrefixPatternMap* prefixPatternMap = new PrefixPatternMap(); 
    vector<RoadSegment*>* latticePatternVec = new vector<RoadSegment*>(); 
    int MaxPosNumber = 0;
    ofstream* logout;

    TDGenerator(TrajectoryDB* tdb, int k, ofstream* fout){
        this->K = k;
        this->trajDB = tdb;
        this->logout = fout;
    }
    
    void startupGenerator(){
        Timer* timer = new Timer();
        timer->resetTimer();
        *logout << "Generator_start" <<endl; 
        scan2SeqPattern();
        *logout << "scanDBfor2-Seq " << timer->getMiliSecond() << endl;
        trimming(Config::MIN_SUPPORT);
        *logout << "trimming " << timer->getMiliSecond() << endl;
        buildPatternMap();
        cout <<"buildPatternMap " << timer->getMiliSecond() << endl;
        *logout << "Generator_end" <<endl; 
        delete timer;
    }

    void addPrefixPattern(PrefixPatternMap* ppm, RoadSegment* segments){
        RoadSegment* rSs = new RoadSegment[segments[0] - 1];
        memcpy(rSs, segments, sizeof(RoadSegment) * (segments[0] - 1));
        rSs[0]--;
        if (ppm->count(rSs) <= 0) {
            PrefixPattern* pp = new PrefixPattern(rSs);
            (*ppm)[pp->roadSegments] = pp;
        }
        (*ppm)[rSs]->sufSegments.push_back(segments);
        delete [] rSs;
    } 

    void addPrefixPattern(RoadSegment* segments){
        addPrefixPattern(this->prefixPatternMap, segments);
    }

    void updatePatternInfo(PatternTD* pat, vector<RoadSegment*>* latPatVec, PrefixPatternMap* ppm) {
        pat->vec2Set();
        // pat->show();
        addPrefixPattern(ppm, pat->roadSegments);
        latPatVec->push_back(pat->roadSegments);
    }

    void trimming(int min_support){
        auto it = this->patternTDMap.begin();
        // cout << this->patternTDMap.size() << endl;
        while (it != this->patternTDMap.end()){
            if (it->second->positions->size() < min_support) {
                PatternTD* ptncopy = it->second;
                it = this->patternTDMap.erase(it); 
                delete ptncopy;
            }
            else{
                updatePatternInfo(it->second, this->latticePatternVec, this->prefixPatternMap);
                ++it;
            } 
        }
        
        *logout << "trimming_finish " << this->patternTDMap.size() << endl;
        *logout << "trimming_finish_prefix_pat " << this->prefixPatternMap->size()  << endl;
    }

    void scan2SeqPattern(){
        RoadSegment preseg; 
        RoadSegment sufseg; 
        RoadSegment* seq = new RoadSegment[3]{3}; 
        int calc_count = 0;
        for(int i = 0; i < this->trajDB->trajsNumber; i++){
            for (int j =1; j < this->trajDB->trajectoryList[i]->length; j++){
                preseg = (RoadSegment) this->trajDB->getSegId(i, j-1);
                sufseg = (RoadSegment) this->trajDB->getSegId(i, j);
                if (j == 1) this->segmentSet.insert(preseg); 
                this->segmentSet.insert(sufseg); 
                seq[1] = preseg; seq[2] = sufseg;
                if (patternTDMap.count(seq) <= 0) {
                    PatternTD* patternTD = new PatternTD(seq);
                    patternTDMap[patternTD->roadSegments] = patternTD;
                } 
                calc_count++;
                patternTDMap[seq]->positions->push_back(pair<int, int>(i, j));
            }
        }
        delete [] seq;
        // cout<<"2-SeqPatterns finish "<<patternMap.size() << ' '<< calc_count <<endl;
    }

    void updateTempLattice(PrefixPatternMap* ppm, vector<RoadSegment*>* vecRS){
        // cout << this->latticePatternVec->size() << ' ' << this->prefixPatternMap->size() << endl;
        delete this->latticePatternVec;
        auto it = this->prefixPatternMap->begin();
        while (it != this->prefixPatternMap->end()){
            delete it->second;
            it++;
        }      
        delete this->prefixPatternMap;

        this->prefixPatternMap = ppm;
        this->latticePatternVec = vecRS;
    }

    void positionsIntersection(PatternTD* prep, PatternTD* sufp, PatternTD* newp) {
        auto first1 = prep->positions->begin();
        auto first2 = sufp->positions->begin();
        pair<int, int> prepos;
        while (first1 != prep->positions->end()&& first2 != sufp->positions->end()){
            prepos.first = first1->first;
            prepos.second = first1->second + 1;
            if (prepos < *first2) ++first1;
            else if (*first2 < prepos) ++first2;
            else {
                newp->positions->push_back(prepos);
                ++first1; ++first2;
            }
        }
    }

    PatternTD* mergePatternPair(RoadSegment* pre, RoadSegment* suf, RoadSegment* new_rSs){
        PatternTD* newp = new PatternTD(new_rSs);
        positionsIntersection(patternTDMap[pre], patternTDMap[suf], newp);
        
        if (newp->positions->size() >= Config::MIN_SUPPORT) {
            newp->roadSegments[newp->roadSegments[0]-1] = suf[suf[0]-1];
            this->patternTDMap[newp->roadSegments] = newp;        
            return newp;
        }
        delete newp;
        return NULL;
    }

    void buildPatternMap(){
        int itr_K = 2;
        while (itr_K < this->K){
            RoadSegment* new_rSs = new RoadSegment[itr_K+2];
            new_rSs[0] = itr_K+2;
            RoadSegment* mid_rSs = new RoadSegment[itr_K];
            mid_rSs[0] = itr_K;
            PrefixPatternMap* new_prefixPatternMap = new PrefixPatternMap(); 
            vector<RoadSegment*>* new_latticePatternVec = new vector<RoadSegment*>(); 

            for(int i = 0; i < this->latticePatternVec->size(); i++){
                RoadSegment* preRSs = this->latticePatternVec->at(i);
                memcpy(mid_rSs+1, preRSs+2, sizeof(RoadSegment) * (preRSs[0]-2));
                memcpy(new_rSs+1, preRSs+1, sizeof(RoadSegment) * (preRSs[0]-1));
                auto it = prefixPatternMap->find(mid_rSs);
                if (it != prefixPatternMap->end()){
                    for(int j = 0; j < it->second->sufSegments.size(); j++){
                        RoadSegment* sufRSs = it->second->sufSegments[j];
                        PatternTD* res = mergePatternPair(preRSs, sufRSs, new_rSs);
                        if (NULL != res ){
                           updatePatternInfo(res, new_latticePatternVec, new_prefixPatternMap);
                        } 
                    }
                } 
            }      

            delete [] mid_rSs;
            delete [] new_rSs;
            updateTempLattice(new_prefixPatternMap, new_latticePatternVec);
            *logout << "Layer" << itr_K << ": " << patternTDMap.size() <<endl;
            itr_K++; 
        }
        updateTempLattice(NULL, NULL);
    }
    ~TDGenerator(){
        // trimming(INT_MAX);
        auto it = this->patternTDMap.begin();
        while (it != this->patternTDMap.end()){
            delete it->second;
            it++;
        }
    }
};

#endif