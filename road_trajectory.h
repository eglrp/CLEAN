#ifndef __TE_road_trajectory__
#define __TE_road_trajectory__

#include <iostream>
#include <vector>

#include "file_processor.h"
#include "pattern.h"
#include "huffman.h"
#include <math.h>

using namespace std;

struct SegPair{
	int xp;
	int ts;
	SegPair(int xp, int ts){
		this->xp = xp;
		this->ts = ts;
	}
};
struct TemporalPair{
	double d;
	int t;
	TemporalPair(int t, double d) {
		this->t = t;
		this->d = d;
	}
	
	void display() {
		cout << t << ":" << d << endl;
	}
};
struct TemporalPairInt {
	int d;
	int t;
	TemporalPairInt(int t, int d) {
		this->t = t;
		this->d = d;
	}
	
	void display() {
		cout << t << ":" << d << endl;
	}
};

class TrajStripe{
public:
	TemporalPair* head;
	TemporalPair* tail;
	vector<SegPair*> * stripe = new vector<SegPair*>();
	TrajStripe(TemporalPair* head, TemporalPair* tail) {
		this->head = head;
		this->tail = tail;
	}
	void display(){
		cout << "size of tuples: " << stripe->size() <<endl; 
		cout << head->d << " " << head->t << endl;
		int acc_t = head->t;
		int acc_d = head->d;
		for (int j=0; j<this->stripe->size(); ++j){
			acc_t += this->stripe->at(j)->ts; 
			acc_d += 10 * ((int)pow(2, this->stripe->at(j)->xp));
			cout << this->stripe->at(j)->xp << " " << this->stripe->at(j)->ts << " "<< acc_d <<" "<< acc_t<< endl;
		}
		cout << tail->d << " " << tail->t << endl;
	}
	int countBit(int n) {
		int count=0,i;
		if(n==0) return 0;
		for(i=0; i< 32; i++)
		{	
			if( (1 << i) & n)
				count=i;
		}
		return ++count;
	}

	vector<bool> convert(int x, int width) {
		vector<bool> ret(width,0);
		int pos = 0;
		while(x) {
			if (x&1)
				ret[pos] = 1;
			else
				ret[pos] = 0;
			x>>=1;  
			pos ++;
		}
		reverse(ret.begin(),ret.end());
		return ret;
	}

	void store(FileWriter* head_fw, int chunk_size, int Unit){
		int pre_xp = 0;
		int pre_ts = 0;
		int max_xp = 0;
		int max_ts = 0;
		int head_t= head->t;
		int head_d = head->d;
		int head_pos = 0;
		vector<int> xp_chunk;
		vector<int> ts_chunk;
		for (int i = 0; i <= stripe->size(); i++){
			if ((i - head_pos) >= chunk_size || i == stripe->size()){
				//cout << head_pos <<" "<< i << endl;
				head_fw->writeInt(head_t);
				head_fw->writeInt(head_d);
				//cout << "Head " << head_t << " " << head_d <<endl;
				int xp_w = countBit(max_xp);
				int ts_w = countBit(max_ts);
				head_fw->writeChar((char) xp_w);
				head_fw->writeChar((char) ts_w);
				vector<bool>* bin_chunk = new vector<bool>();
				for (int j = 0; j < xp_chunk.size(); j++){
					//cout<< xp_chunk[j] << " " << ts_chunk[j] << endl;
					vector<bool> bs_xp = convert(xp_chunk[j], xp_w);
					vector<bool> bs_ts = convert(ts_chunk[j], ts_w);
					bin_chunk->insert(bin_chunk->end(), bs_xp.begin(), bs_xp.end()); 
					bin_chunk->insert(bin_chunk->end(), bs_ts.begin(), bs_ts.end());
				}
				Binary* binary = new Binary(bin_chunk);
				binary->store(head_fw);
				if (i == stripe->size()) break;
				head_t = head->t + stripe->at(i)->ts;
				head_d = head->d + stripe->at(i)->xp * Unit - stripe->at(i)->ts;
				max_xp = 0;
				max_ts = 0;
				pre_xp = stripe->at(i)->xp;
				pre_ts = stripe->at(i)->ts;
				head_pos = i + 1;
				xp_chunk.clear();
				ts_chunk.clear();
				// cout << i << endl;
				continue;
			}
			if (stripe->at(i)->xp - pre_xp > max_xp) max_xp = stripe->at(i)->xp - pre_xp;
			if (stripe->at(i)->ts - pre_ts > max_ts) max_ts = stripe->at(i)->ts - pre_ts;
			xp_chunk.push_back(stripe->at(i)->xp - pre_xp);
			ts_chunk.push_back(stripe->at(i)->ts - pre_ts);
			pre_xp = stripe->at(i)->xp;
			pre_ts = stripe->at(i)->ts;
			
		}

		head_fw->writeInt(tail->t);
		head_fw->writeInt(tail->d);
		head_fw->writeChar((char) -1 );
		head_fw->writeChar((char) -1 );
	}

	void display(ofstream* fout){
		cout << "size of tuples: " << stripe->size() <<endl; 
		*fout << head->t << " " << head->d << endl;
		int acc_t = head->t;
		int acc_d = head->d;
		for (int j=0; j < this->stripe->size(); ++j){
			acc_t += this->stripe->at(j)->ts; 
			// acc_d += 10 * this->stripe->at(j)->xp;
			*fout << this->stripe->at(j)->ts << " " << head->d + this->stripe->at(j)->xp * 10 - this->stripe->at(j)->ts << endl;
		}
		*fout << tail->t << " " << tail->d << endl;
	}
};


class Trajectory{
public:
    int id;
    vector<int> segmentList;
    int* cprsMarks;
    int length;
    vector<int> blockList;
};




class TrajectoryDB{
public:
    int trajsNumber;
    vector<Trajectory*> trajectoryList;
    map<int, int> frequency;
    TrajectoryDB(FileReader* fr, int trajsNeeded){
        this->trajsNumber = 0;
        int tmpId = 0;
        int trajs_size = fr->nextInt();
        while ((tmpId = fr->nextInt()) != EOF){
            ++this->trajsNumber;
            Trajectory * traj = new Trajectory();
            traj->id = this->trajsNumber;
            traj->length = tmpId;
            traj->cprsMarks = new int[traj->length]();
            for (int i = 0; i < traj->length; i++)
                traj->segmentList.push_back(fr->nextInt());
            this->trajectoryList.push_back(traj);
            if (this->trajsNumber == trajsNeeded) break;
        } 
    }

    int pack4chars(char c1, char c2, char c3, char c4) {
        return ((int)(((unsigned char)c1) << 24)
            |  (int)(((unsigned char)c2) << 16)
            |  (int)(((unsigned char)c3) << 8)
            |  (int)((unsigned char)c4));
    }

    TrajectoryDB(FileReader* fr, int trajsNeeded, bool isText){
        this->trajsNumber = 0;
        char tmpId;
        vector<char> tmpRow;
        while ((tmpId = fr->nextChar()) != EOF){
            ++this->trajsNumber;
            Trajectory * traj = new Trajectory();
            traj->id = this->trajsNumber;
            tmpRow.clear();
            while (true){
                tmpRow.push_back(tmpId);
                if (tmpId == '\n' || tmpId == EOF) break;
                tmpId = fr->nextChar();
            }

            traj->length = tmpRow.size()/4;
            if (tmpRow.size() % 4 != 0){
                traj->length++;
                for (int _i = 0; _i < 4 - tmpRow.size() % 4; _i++)
                    tmpRow.push_back((char)0);
            }   
            
            traj->cprsMarks = new int[traj->length]();
            for (int i = 0; i < traj->length; i++){
                traj->segmentList.push_back(
                    pack4chars(tmpRow[i*4], tmpRow[i*4 + 1], tmpRow[i*4 + 2], tmpRow[i*4 + 3])
                );
                //cout << tmpRow[i];
            }
            // cout <<endl;
            this->trajectoryList.push_back(traj);
            if (this->trajsNumber == trajsNeeded) break;
        }
    }

    int getSegId(int x, int y){
        return this->trajectoryList[x]->segmentList[y];
    }

    void getSlice(int tid, int left, int right, RoadSegment* rs){
        rs[0] = right - left + 2;
        for (int i = left; i <= right; i++)
            rs[i-left+1] =(RoadSegment) this->getSegId(tid, i);
    }

    void compressedTrajStore(HuffmanTree* huffman, FileWriter* binWriter){
        vector<bool>* code;
        Binary* binary;
        
        for (int i = 0; i < this->trajsNumber; i++){
            code = new vector<bool>(); 
            for (int j = 0; j < this->trajectoryList[i]->blockList.size(); j++) {
                HuffmanCode* item = huffman->blockHuffmanCode[this->trajectoryList[i]->blockList[j]];
                copy(item->code.begin(), item->code.end(), back_inserter(*code));
            }
            binary = new Binary(code);
            binary->store(binWriter);
            delete binary;
        }
		
    }

    ~TrajectoryDB(){
        for(int i = 0; i < trajsNumber; i++)
            delete trajectoryList[i];
    }
};

class RoadNetTrajectory {
public:
	int spatialNumber, temporalNumber;
    double unit_length;
	vector<int>* spatial = new vector<int>();
	vector<TemporalPair*>* temporal = new vector<TemporalPair*>();
	
	//Construct a road network trajectory from Spatial File Reader and Temporal File Reader
	RoadNetTrajectory(FileReader* spatialReader, FileReader* temporalReader) {
		//Construct spatial path
		this->spatialNumber = spatialReader->nextInt();
		this->spatial->clear();
		for (int i = 0; i < this->spatialNumber; ++i) {
			this->spatial->push_back(spatialReader->nextInt());
		}
		//Construct temporal sequence
		this->temporalNumber = temporalReader->nextInt();
		this->temporal->clear();
		for (int i = 0; i < this->temporalNumber; ++i) {
			this->temporal->push_back(new TemporalPair(temporalReader->nextInt(), temporalReader->nextDouble()));
		}
	}
	
	//Construct a trajectory from spatial and temporal vector
	RoadNetTrajectory(vector<int>* spatial, vector<TemporalPair*>* temporal) {
		this->spatial = spatial;
		this->spatialNumber = (int)spatial->size();
		this->temporal = temporal;
		this->temporalNumber = (int)temporal->size();
	}
	
	//Store the road network trajectory
	void store(FileWriter* spatialWriter, FileWriter* temporalWriter) {
		spatialWriter->writeInt(this->spatialNumber);
		for (int i = 0; i < this->spatialNumber; ++i) {
			spatialWriter->writeInt(spatial->at(i));
		}
		temporalWriter->writeInt(this->temporalNumber);
		for (int i = 0; i < this->temporalNumber; ++i) {
			temporalWriter->writeInt(temporal->at(i)->t);
			temporalWriter->writeDouble(temporal->at(i)->d);
		}
	}
	
	//Display the road network trajectory
	void display() {
		cout << "Spatial" << endl;
		cout << this->spatialNumber << endl;
		for (int i = 0; i < this->spatialNumber; ++i) {
			cout << this->spatial->at(i) << " ";
		}
		cout << endl << endl;
		
		cout << "Temporal" << endl;
		cout << this->temporalNumber << endl;
		for (int i = 0; i < this->temporalNumber; ++i) {
			cout << ", " << this->temporal->at(i)<< "\t    ";
		}
		cout << endl;
	}
	
	~RoadNetTrajectory() {
		delete spatial;
		delete temporal;
	}
};

#endif // __TE_road_trajectory__
