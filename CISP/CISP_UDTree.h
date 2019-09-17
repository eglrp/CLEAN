#ifndef __TE_CISP_UpDownTree__
#define __TE_CISP_UpDownTree__

#include <vector>
#include <set>

#include "CISP_MSFI.h"
#include "road_trajectory.h"

using namespace std;

class TreeNode {
public:
    vector<int> sidList;
    TreeNode* father;
    map<int,TreeNode*> sonMap; 
    set<int> tidSet;
    TreeNode(TreeNode* fa, int sid){
        this->father = fa;
        this->sidList.push_back(sid);
    }
    TreeNode* insertNode(int sid, bool isLast, int tid){
        if (sonMap[sid] == NULL)
            sonMap[sid] = new TreeNode(this, sid);
        if (isLast)
            this->tidSet.insert(tid);
        return sonMap[sid];
    }
};

class UDTree {
public:
    TreeNode* uptree;
    TreeNode* downtree;
    TrajectoryDB* trajDB;
    int core;
    UDTree(TrajectoryDB* trajDB, int root_sid){
        this->uptree = new TreeNode(NULL, root_sid);
        this->downtree = new TreeNode(NULL, root_sid);
        this->core = root_sid;
        this->trajDB = trajDB;
    }
    void buildUDTree(MSFI* msfi){
        for(int i = 0; i < msfi->MSFIP[core].size(); i++)
            insertSeq2UpTree(msfi->MSFIP[core][i]);
        for(int i = 0; i < msfi->MSFIS[core].size(); i++)
            insertSeq2DownTree(msfi->MSFIS[core][i]);
    }
    void insertSeq2UpTree(Subsequence* subseq){
        TreeNode* currentNode = this->uptree;
        int sid = 0;
        for(int i = subseq->end - 1; i >= subseq->start; i--){
            sid = trajDB->getSegId(subseq->tid, i);
            if (i == subseq->start)
                currentNode = currentNode->insertNode(sid, true, subseq->tid);
            else
                currentNode = currentNode->insertNode(sid, false, 0);
        } 
    } 
    void insertSeq2DownTree(Subsequence* subseq){
        TreeNode* currentNode = this->downtree;
        int sid = 0;
        for(int i = subseq->start + 1; i <= subseq->end; i++){
            sid = trajDB->getSegId(subseq->tid, i);
            if (i == subseq->end)
                currentNode = currentNode->insertNode(sid, true, subseq->tid);
            else
                currentNode = currentNode->insertNode(sid, false, 0);
        } 
 
    }
};

#endif