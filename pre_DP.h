
#ifndef PRESS_v2_pre_processing_h
#define PRESS_v2_pre_processing_h

#include "utility.h"
#include "file_processor.h"
#include "road_network.h"
#include "road_trajectory.h"
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace std;

// Point map result is tuple (t, edgeId)
struct PointMatch {
	int t, edgeId;
	double lat, lng;
	double proj_lat, proj_lng;
	double location;
	PointMatch(int t, double lat, double lng, int edgeId, double proj_lat, double proj_lng, double location) {
		this->t = t;
		this->edgeId = edgeId;
		this->lat = lat;
		this->lng = lng;
		this->proj_lat = proj_lat;
		this->proj_lng = proj_lng;
		this->location = location;
	}
};


// Map match result sequence
class MapMatchResult {
public:
	vector<PointMatch*>* sequence = new vector<PointMatch*>();
	
	//Construct a Map Match result from a file handler
	MapMatchResult(FileReader* fr) {
		this->sequence->clear();
		int t;
		while ((t = fr->nextInt()) != EOF) {
			fr->nextChar();
			double lat = fr->nextDouble();
			fr->nextChar();
			double lng = fr->nextDouble();
			fr->nextChar();
			int edgeId = fr->nextInt();
			fr->nextChar();
			double proj_lat = fr->nextDouble();
			fr->nextChar();
			double proj_lng = fr->nextDouble();
			fr->nextChar();
			double location =fr->nextDouble();
			fr->nextChar(); fr->nextDouble();
			// if (this->sequence->size() > 0 && edgeId == this->sequence->back()->edgeId && (location - this->sequence->back()->location < 0.05)){
			// 	continue
			// }
			this->sequence->push_back(new PointMatch(t, lat, lng, edgeId, proj_lng, proj_lng, location));
		}
	}
	
	// get processed edge sequence
	vector<int>* getProcessedSequence() {
		vector<int>* result = new vector<int>();
		int i = 0;
		while (i < sequence->size() && sequence->at(i)->edgeId == Config::NULL_POINTER) {
			i++;
		}
		for (; i < sequence->size(); ++i) {
			if (sequence->at(i)->edgeId != Config::NULL_POINTER) {
				if (!result->size() || result->at(result->size() - 1) != sequence->at(i)->edgeId) {
					result->push_back(sequence->at(i)->edgeId);
				}
			}
		}
		return result;
	}
	
	//Display a GPS trajectory
	void display() {
		for (int i = 0; i < sequence->size(); ++i) {
			cout << sequence->at(i)->t << ": " << sequence->at(i)->edgeId << endl;
		}
	}
	
	~MapMatchResult() {
		for (int i = 0; i < sequence->size(); ++i) {
			delete sequence->at(i);
		}
		delete sequence;
	}
};


class PreProcessor {
private:
	static PreProcessor* instance;
	
public:
	static PreProcessor* getInstance() {
		if (instance == NULL) {
			instance = new PreProcessor();
		}
		return instance;
	}
	
	bool pave_fitting(int delta, int j, double Head,double Head_d, vector<TemporalPair*>* tempPairs, double travel_sofar, int last_ts, int Unit_D_T){
		if (delta + travel_sofar > tempPairs->back()->t + tempPairs->back()->d - Head){
			return false;
		}
		int j_at_right = j;
		while (j_at_right < tempPairs->size()-1 && tempPairs->at(j_at_right)->d + tempPairs->at(j_at_right)->t - Head <= delta + travel_sofar) {
			j_at_right++;
		}
		int _t2 = tempPairs->at(j_at_right - 1)->t;
		double _d2 = tempPairs->at(j_at_right - 1)->d;
		int t2 = tempPairs->at(j_at_right)->t;
		double d2 = tempPairs->at(j_at_right)->d;
		
		int dis_sofar = travel_sofar - last_ts;
		double remain_dis = d2 - Head_d - dis_sofar;
		int ts = _t2 + (t2 - _t2)*(delta + travel_sofar + Head - _t2 - _d2 )/(d2+t2 - _t2 - _d2);
		int del_ts = ts -(Head - Head_d) - last_ts;
		
		double del_dis = delta - del_ts;
		for (int mid= j; mid < j_at_right; mid++){
			double inner_d_offset = tempPairs->at(mid)->d - Head_d - dis_sofar;
			double inner_t_offset = tempPairs->at(mid)->t - (Head - Head_d) - last_ts;
			double interpolate_t = del_ts * inner_d_offset / del_dis;
			double interpolate_d = del_dis * inner_t_offset / del_ts;
			if (abs(inner_d_offset - interpolate_d) > Unit_D_T) 
			// if (abs(inner_d_offset - interpolate_d) > 1) 
				return false;
			if (abs(inner_t_offset - interpolate_t) > Unit_D_T)
			// if (abs(inner_t_offset - interpolate_t) > 1)
				return false;
		}
		return true;
	}

	vector<SegPair*>* DauglasPeucker(vector<TemporalPair*>* temps, int l, int r, int l_t, double l_d, int r_t, double r_d, double epsilon){
		int pos = -1;
		double score = -9999;
		bool enough = true;
		vector<SegPair*>* vec_res = new vector<SegPair*>();
		// cout << l << " "<< r << " "<< " "<<endl;
		if (l > r) {
			return vec_res;
		}
		double sum_l = l_t + l_d;
		double sum_r = r_t + r_d;
		for (int i = l; i <= r; i++){
			int t = temps->at(i)->t;
			double d = temps->at(i)->d;
			double interpo_d = l_d + (r_d - l_d) * (t - l_t) / (r_t - l_t);
			double interpo_t = l_t + (r_t - l_t) * (d - l_d) / (r_d - l_d);
			double err_d = abs(d - interpo_d);
			double err_t = abs(t - interpo_t);
			if (err_d + err_t > score){
				score = err_d + err_t;
				pos = i;
			}
			if (err_d > epsilon || err_t > epsilon){
				enough = false;
			}
		}
		if (not enough){
			double sum_dt = temps->at(pos)->d + temps->at(pos)->t;
			double sum_dt_pre = temps->at(pos - 1)->d + temps->at(pos - 1)->t;
			double sum_dt_aft = temps->at(pos + 1)->d + temps->at(pos + 1)->t;
			int beta = (int) sum_dt / epsilon;
			
			double insert_sum = 0;
			int insert_t = 0;
			if (abs(beta * epsilon - sum_dt) < abs((beta+1) * epsilon - sum_dt)){
				insert_sum = beta * epsilon;
				insert_t = (int)(temps->at(pos)->t - (temps->at(pos)->t - temps->at(pos - 1)->t)*(sum_dt - insert_sum)/(sum_dt - sum_dt_pre));	
			} else {
				insert_sum = (beta + 1) * epsilon;
				beta++;
				insert_t = (int)(temps->at(pos)->t + (temps->at(pos + 1)->t - temps->at(pos)->t)*(insert_sum - sum_dt)/(sum_dt_aft- sum_dt));	
			}

			SegPair* sp = new SegPair(beta, insert_t);
			vector<SegPair*>* vec1 = DauglasPeucker(temps, l, pos - 1, l_t, l_d, insert_t, insert_sum - insert_t, epsilon);
			vector<SegPair*>* vec2 = DauglasPeucker(temps, pos + 1, r, insert_t, insert_sum - insert_t, r_t, r_d, epsilon);
			vec_res->insert(vec_res->end(), vec1->begin(), vec1->end()); 
			vec_res->push_back(sp);
			vec_res->insert(vec_res->end(), vec2->begin(), vec2->end()); 
			// delete vec1;
			// delete vec2;
		}
		// if (enough)
		// 	cout << l << " "<< r << " "<< r-l+1<< endl;
		return vec_res;
	}

	// generate spatial
	// get GPS path set of one day from gpsTraPath, get map match result from mapmatchTraSet
	// write Spatial and Temporal component to spatialWriter and temporalWriter
	void generateRoadNetTrajectory(Graph* graph, char* mapmatchTraPath, char* spatialPath, char* temporalPath) {
		vector<char*>* mapmatchPathSet = FileTool::getInstance()->getFileNameSet(mapmatchTraPath);
		
		if (mapmatchPathSet->size() ==  0) {
			throw "No trajectories";
		}
		
		FileWriter* spatialWriter = new FileWriter(spatialPath, true);
		FileWriter* temporalWriter = new FileWriter(temporalPath, true);
		
		spatialWriter->writeInt((int)mapmatchPathSet->size());
		temporalWriter->writeInt((int)mapmatchPathSet->size());
		int origin_volume = 0;
		int cprs_volume = 0;
		for (int i = 0; i < mapmatchPathSet->size(); ++i) {
			
			FileReader* mapReader = new FileReader(mapmatchPathSet->at(i), false);
			MapMatchResult* mm = new MapMatchResult(mapReader);
			delete mapReader;

			// Hook: outlier trajectory, directly match to 0
			vector<int>* processedSequence = mm->getProcessedSequence();
			if (!processedSequence->size()) {
				continue;
			}
			if (mm->sequence->size() < 30) continue; 
			// generate temporal component
			for (int j = 1; j < mm->sequence->size(); ++j) {
				if (mm->sequence->at(j)->edgeId == mm->sequence->at(j - 1)->edgeId &&
					mm->sequence->at(j)->location < mm->sequence->at(j - 1)->location) {
					mm->sequence->at(j)->location = mm->sequence->at(j - 1)->location;
				}
			}
			
			for (int j = (int)mm->sequence->size() - 2; j >= 0; --j) {
				if (mm->sequence->at(j)->edgeId == Config::NULL_POINTER) {
					mm->sequence->at(j)->edgeId = mm->sequence->at(j + 1)->edgeId;
				}
			}
			
			double accumulate = 0;			// distance accumulation
			int pt = 0;						// pointer to the edge
			vector<int> * spatial = new vector<int>();
			vector<TemporalPair*>* dtPairs = new vector<TemporalPair*>();

			double dis_sofar = 0;
			int obs_cnt = 0; // count the number of observation gps points	
			for (int j = 0; j < mm->sequence->size(); ++j) {
				int len = graph->getEdge(mm->sequence->at(j)->edgeId)->len;
				if (mm->sequence->at(j)->proj_lng != 0){
					dis_sofar = accumulate + len * mm->sequence->at(j)->location;
					if (mm->sequence->at(j)->t <= 0 || dis_sofar <= 0) continue;
					if (dtPairs->size() > 0 && dtPairs->back()->t > mm->sequence->at(j)->t) continue;
					if (dtPairs->size() > 0 && dtPairs->back()->d > dis_sofar) continue;
					dtPairs->push_back(new TemporalPair(mm->sequence->at(j)->t, dis_sofar));
					//*fout_org << tempPairs->back()->t<< " "<< tempPairs->back()->d<< endl;
					obs_cnt ++;
				}
				if ((j== mm->sequence->size()-1) ||(mm->sequence->at(j+1)->edgeId != mm->sequence->at(j)->edgeId)){
					spatial->push_back(mm->sequence->at(j)->edgeId);
					accumulate += len;
				}
			}
			if (spatial->size() < 30 || dtPairs->size() < 30) continue;
			RoadNetTrajectory* trajectory = new RoadNetTrajectory(spatial, dtPairs);
			trajectory->store(spatialWriter, temporalWriter);
			cout << mapmatchPathSet->at(i)<<" traj_size: " << spatial->size() << " " << dtPairs->size() << endl;
			delete trajectory;
		}
		// cout << origin_volume << " " << cprs_volume << endl;		
		delete mapmatchPathSet;
		delete spatialWriter;
		delete temporalWriter;
	}
};

PreProcessor* PreProcessor::instance = NULL;

#endif
