#ifndef __TE_BU_Generator__
#define __TE_BU_Generator__

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "road_trajectory.h"
#include "pattern_BU.h"

using namespace std;

typedef map<RoadSegment*, NeighborBU* , arr_cmp> InnerMap;
typedef map<RoadSegment*, InnerMap , arr_cmp > NeighborMap;
typedef map<RoadSegment*, PatternBU*, arr_cmp> PatternBUMap;
typedef pair<RoadSegment*, RoadSegment*> SeqPair;
typedef map<RoadSegment*, vector<SeqPair>, arr_cmp> ParentMap;

class BUGenerator{
public:
    NeighborMap nbrLeftMap ;
    NeighborMap nbrRightMap;
    PatternBUMap patternBUMap;
    TrajectoryDB* trajDB;
    set<RoadSegment> segmentSet;
    ParentMap parentMap;
    int K;

    BUGenerator(TrajectoryDB* tdb, int k){
        this->trajDB = tdb;
        this->K = k;
        initScan();
    }

    RoadSegment* checkOneSide(NeighborMap& nbr, RoadSegment* rss, int& max){
        RoadSegment* res = NULL;
        int support = 0;
        int mergeWidth = 0;
        for (auto it = nbr[rss].begin(); it != nbr[rss].end(); it++){
            NeighborBU* nbu = nbr[rss][it->first];
            support = nbu->positions->size();
            mergeWidth = rss[0]+it->first[0]-2;
            if (mergeWidth <= this->K && 2 <= support && support > max){
                max = support;
                res = it->first;
            } 
        }
        return res;
    }

    RoadSegment* getMaxNeighbor(RoadSegment* rss, bool& isLeft){
        int max = 0;
        RoadSegment* resL = NULL;
        RoadSegment* resR = NULL;
        resL = checkOneSide(nbrLeftMap, rss, max);
        resR = checkOneSide(nbrRightMap, rss, max);
        isLeft =  (resR == NULL);
        return isLeft ? resL : resR;         
    }

    RoadSegment* generateNewKey(RoadSegment* rssL, RoadSegment* rssR) {
        int mergeLen = rssL[0]+rssR[0]-1;
        RoadSegment* merge_key =new RoadSegment[mergeLen]();
        merge_key[0] = mergeLen; 
        memcpy(merge_key+1, rssL+1, sizeof(RoadSegment)*(rssL[0]-1));
        memcpy(merge_key+rssL[0], rssR+1, sizeof(RoadSegment)*(rssR[0]-1));
        return merge_key;
    }


    void pos_set_difference(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2, int bias) {
        vector<pair<int, int>>* new_pos = new vector<pair<int, int> >(pos1->size());
        // vector<pair<int, int>>& vecRef = *new_pos;
        auto first1 = pos1->begin();
        auto last1 = pos1->end();
        auto first2 = pos2->begin();
        auto last2 =pos2->end();
        auto result = new_pos->begin();
        pair<int, int> first2_bias;
        while (first1!=last1 && first2!=last2){
            first2_bias.first = first2->first;
            first2_bias.second = first2->second + bias;
            if (*first1<first2_bias) { *result = *first1; ++result; ++first1; }
            else if (first2_bias<*first1) ++first2;
            else { ++first1; ++first2; }
        }
        vector<pair<int, int>>::iterator it;
        it = copy(first1,last1,result);
        new_pos->resize(it - new_pos->begin());  
        delete pos1;
        pos1 = new_pos;
    }

    void pos_set_union(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2, int bias) {
        vector<pair<int, int>>* new_pos = new vector<pair<int, int> >(pos1->size()+pos2->size());
        // vector<pair<int, int>>& vecRef = *new_pos;
        auto first1 = pos1->begin();
        auto last1 = pos1->end();
        auto first2 = pos2->begin();
        auto last2 =pos2->end();
        auto result = new_pos->begin();
        vector<pair<int, int>>::iterator it;
        pair<int, int> first2_bias;
        while (true)
        {
            
            if (first1==last1) {
                while (first2 != last2){
                    first2_bias.first = first2->first;
                    first2_bias.second = first2->second + bias;
                    *result = first2_bias;
                    ++result;
                    ++first2;
                }
                it = result;
                break;
            } 
            if (first2==last2) {
                it =std::copy(first1,last1,result); 
                break;
            } 
            first2_bias.first = first2->first;
            first2_bias.second = first2->second + bias;
            if (*first1<first2_bias) { *result = *first1; ++first1; }
            else if (first2_bias<*first1) { *result = first2_bias; ++first2; }
            else { *result = *first1; ++first1; ++first2; }
            ++result;
        }
        new_pos->resize(it - new_pos->begin());  
        delete pos1;
        pos1 = new_pos;
    }


    void differencePos(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2, int bias){
        // pos_set_difference(pos1, pos2, bias);
        // return;
        vector<pair<int, int>>* new_pos = new vector<pair<int, int>>(pos1->size());
        pair<int, int>* buffer;
        vector<pair<int, int>>::iterator it;
        if (bias == 0)
            it = set_difference(pos1->begin(), pos1->end(), pos2->begin(), pos2->end(), new_pos->begin());
        else{
            buffer = new pair<int, int>[pos2->size()];
            copy(pos2->begin(),pos2->end(), buffer);
            for (int i = 0; i < pos2->size(); i++)
                buffer[i].second += bias;
            it = set_difference(pos1->begin(), pos1->end(), buffer, buffer + pos2->size(), new_pos->begin());
        }
        new_pos->resize(it - new_pos->begin());  
        delete pos1;
        pos1 = new_pos;
    }

    void differencePos(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2){
        differencePos(pos1, pos2, 0);
    }

    void unionPos(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2, int bias){
        // pos_set_union(pos1, pos2, bias);
        // return;
        vector<pair<int, int>>* new_pos = new vector<pair<int, int>>(pos1->size()+pos2->size());
        pair<int, int>* buffer;
        vector<pair<int, int>>::iterator it;
        if (bias == 0)
            it = set_union(pos1->begin(), pos1->end(), pos2->begin(), pos2->end(), new_pos->begin());
        else{
            buffer = new pair<int, int>[pos2->size()];
            copy(pos2->begin(),pos2->end(), buffer);
            for (int i = 0; i < pos2->size(); i++)
                buffer[i].second += bias;
            it = set_union(pos1->begin(), pos1->end(), buffer, buffer + pos2->size(), new_pos->begin());
        }
        new_pos->resize(it - new_pos->begin());  
        delete pos1;
        pos1 = new_pos;
    }
    
    void unionPos(vector<pair<int, int>>*& pos1, vector<pair<int, int>>*& pos2){
        unionPos(pos1, pos2, 0);
    }

    void checkOrCreatePattern(RoadSegment* seq){
        if (this->patternBUMap.count(seq) == 0) {
            PatternBU* pattern = new PatternBU(seq);
            this->patternBUMap[pattern->roadSegments] = pattern;
        }
    }

    RoadSegment* merge(RoadSegment* rssL, RoadSegment* rssR){
        RoadSegment* merge_key = generateNewKey(rssL, rssR);
        differencePos(patternBUMap[rssL]->positions, nbrLeftMap[rssR][rssL]->positions);
        differencePos(patternBUMap[rssR]->positions, nbrRightMap[rssL][rssR]->positions);
        checkOrCreatePattern(merge_key);
        unionPos(patternBUMap[merge_key]->positions, nbrRightMap[rssL][rssR]->positions); 
        parentMap[merge_key].push_back(SeqPair(rssL, rssR));

        for (auto it = nbrLeftMap[rssL].begin(); it != nbrLeftMap[rssL].end(); it ++){
            vector<pair<int, int>> intersectVec; 
            vector<pair<int, int>>* intersect = &intersectVec;; 
            
            set_intersection(nbrLeftMap[rssR][rssL]->positions->begin(), nbrLeftMap[rssR][rssL]->positions->end(),
                  nbrRightMap[it->first][rssL]->positions->begin(), nbrRightMap[it->first][rssL]->positions->end(),
                             inserter(intersectVec,intersectVec.begin()));
            if (intersect->size()==0) continue;
            differencePos(nbrRightMap[it->first][rssL]->positions, intersect);
            checkNbrMap(it->first, merge_key, nbrRightMap);
            unionPos(nbrRightMap[it->first][merge_key]->positions, intersect, rssR[0] - 1);
            checkNbrMap(merge_key, it->first, nbrLeftMap);
            unionPos(nbrLeftMap[merge_key][it->first]->positions, intersect, -rssL[0] + 1);
            differencePos(nbrLeftMap[rssL][it->first]->positions, intersect, -rssL[0] + 1);
        }

        for(auto it = nbrRightMap[rssR].begin(); it != nbrRightMap[rssR].end(); it++){
            vector<pair<int, int>> intersectVec; 
            vector<pair<int, int>>* intersect = &intersectVec;; 
            set_intersection(nbrRightMap[rssL][rssR]->positions->begin(), nbrRightMap[rssL][rssR]->positions->end(),
                  nbrLeftMap[it->first][rssR]->positions->begin(), nbrLeftMap[it->first][rssR]->positions->end(),
                             inserter(intersectVec,intersectVec.begin()));
            if (intersect->size()==0) continue;
            differencePos(nbrLeftMap[it->first][rssR]->positions, intersect);
            checkNbrMap(it->first, merge_key, nbrLeftMap);
            unionPos(nbrLeftMap[it->first][merge_key]->positions, intersect);
            checkNbrMap(merge_key, it->first, nbrRightMap);
            unionPos(nbrRightMap[merge_key][it->first]->positions, intersect, it->first[0]-1);
            differencePos(nbrRightMap[rssR][it->first]->positions, intersect, it->first[0]-1);
        }
        nbrLeftMap[rssR][rssL]->positions->clear();
        nbrRightMap[rssL][rssR]->positions->clear();

        return patternBUMap[merge_key]->roadSegments;
    }

    void addPatternMap(RoadSegment* seq, pair<int, int> pr){
        this->segmentSet.insert(seq[1]); 
        checkOrCreatePattern(seq);
        this->patternBUMap[seq]->positions->push_back(pr);
    }
    
    void checkNbrMap(RoadSegment* fst, RoadSegment* sed, NeighborMap& nbm){
         if (!(nbm.count(fst) > 0 && nbm[fst].count(sed) > 0)){
            NeighborBU* nbr = new NeighborBU(fst, sed);
            nbm[nbr->fstRS][nbr->sedRS] = nbr;
         }
    }

    void addNeighborMap(RoadSegment* pre, RoadSegment* suf, 
                            pair<int, int> prepos, pair<int, int> sufpos){
        checkNbrMap(pre, suf, nbrRightMap);
        nbrRightMap[pre][suf]->positions->push_back(sufpos);
        checkNbrMap(suf, pre, nbrLeftMap);
        nbrLeftMap[suf][pre]->positions->push_back(prepos);
    }

    void initScan(){

        RoadSegment* preseq = new RoadSegment[2]{2}; 
        RoadSegment* sufseq = new RoadSegment[2]{2}; 

        for(int i = 0; i < this->trajDB->trajsNumber; i++){
            for (int j =1; j < this->trajDB->trajectoryList[i]->length; j++){
                preseq[1] = (RoadSegment) this->trajDB->getSegId(i, j-1);
                sufseq[1] = (RoadSegment) this->trajDB->getSegId(i, j);
                pair<int, int> prepos(i, j-1);
                pair<int, int> sufpos(i, j);
                if (j == 1) addPatternMap(preseq, prepos);
                addPatternMap(sufseq, sufpos);
                addNeighborMap(preseq, sufseq, prepos, sufpos);
            }
        }
    } 

    ~BUGenerator(){
        auto it  = this->patternBUMap.begin();
        while (it != this->patternBUMap.end()){
            delete it->second;
            it++;
        }      
    }
};

#endif
