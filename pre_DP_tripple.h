
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

struct SegPair{
	int xp;
	int delta_ts;
	SegPair(int xp, int delta_ts){
		this->xp = xp;
		this->delta_ts = delta_ts;
	}
};

struct TemporalPair {
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
		double acc_d = 0;
		for (int j=0; j<this->stripe->size(); ++j){
			// acc_d += 10 * this->stripe->at(j)->xp;
			
			cout <<this->stripe->at(j)->xp - acc_d <<" "<<this->stripe->at(j)->delta_ts - acc_t<< endl;
			// <<" "<< this->stripe->at(j)->xp * 10 - this->stripe->at(j)->delta_ts << " " <<  this->stripe->at(j)->delta_ts << endl;
			acc_t = this->stripe->at(j)->delta_ts; 
			acc_d = this->stripe->at(j)->xp;
		}
		cout << tail->d << " " << tail->t << endl;
	}
	void display(ofstream* fout){
		cout << "size of tuples: " << stripe->size() <<endl; 
		*fout << head->t << " " << head->d << endl;
		int acc_t = head->t;
		double acc_d = head->d;
		for (int j=0; j<this->stripe->size(); ++j){
			acc_t += this->stripe->at(j)->delta_ts; 
			// acc_d += 10 * this->stripe->at(j)->xp;
			*fout << this->stripe->at(j)->delta_ts << " " <<  head->d + this->stripe->at(j)->xp * 10 - this->stripe->at(j)->delta_ts << endl;
		}
		*fout << tail->t << " " << tail->d << endl;
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

	void calc_insert(vector<TemporalPair*>* temps, int pos, double & insert_sum, int & insert_t,int &beta, double epsilon){
		double sum_dt = temps->at(pos)->d + temps->at(pos)->t;
		double sum_dt_pre = temps->at(pos - 1)->d + temps->at(pos - 1)->t;
		double sum_dt_aft = temps->at(pos + 1)->d + temps->at(pos + 1)->t;
		beta = (int) sum_dt / epsilon;
		if (abs(beta * epsilon - sum_dt) < abs((beta+1) * epsilon - sum_dt)){
			insert_sum = beta * epsilon;
			insert_t = (int)(temps->at(pos)->t - (temps->at(pos)->t - temps->at(pos - 1)->t)*(sum_dt - insert_sum)/(sum_dt - sum_dt_pre));	
		} else {
			insert_sum = (beta + 1) * epsilon;
			beta++;
			insert_t = (int)(temps->at(pos)->t + (temps->at(pos + 1)->t - temps->at(pos)->t)*(insert_sum - sum_dt)/(sum_dt_aft- sum_dt));	
		}
	}

	vector<SegPair*>* DauglasPeucker(vector<TemporalPair*>* temps, int l, int r, int l_t, double l_d, int r_t, double r_d, double epsilon){
		int pos_d = -1;
		int pos_t = -1;
		double diff_d = -9999;
		double diff_t = -9999;
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
			if (err_d > diff_d){
				diff_d = err_d;
				pos_d = i;
			}
			if (err_t > diff_t){
				diff_t = err_t;
				pos_t = i;
			}
			if (err_d > epsilon || err_t > epsilon){
				enough = false;
			}
		}
		int pos1 = pos_t;
		int pos2 = pos_d;
		if (pos_t > pos_d){
			pos1 = pos_d;
			pos2 = pos_t;	
		}
		cout<< pos1 << " " << pos2<< endl;
		if (not enough){
			double insert_sum = 0;
			int insert_t = 0;
			int beta;
			calc_insert(temps, pos1, insert_sum, insert_t, beta, epsilon);	
			vector<SegPair*>* vec1 = DauglasPeucker(temps, l, pos1 - 1, l_t, l_d, insert_t, insert_sum - insert_t, epsilon);
			SegPair* sp = new SegPair(beta, insert_t);
			vec_res->insert(vec_res->end(), vec1->begin(), vec1->end()); 
			vec_res->push_back(sp);
			if (pos2 > pos1){
				double insert_sum_2 = 0;
				int insert_t_2 = 0;
				int beta_2;
				calc_insert(temps, pos2, insert_sum_2, insert_t_2, beta_2, epsilon);
				SegPair* sp_2 = new SegPair(beta_2, insert_t_2);
				vector<SegPair*>* vec2 = DauglasPeucker(temps, pos1 + 1, pos2 - 1, insert_t, insert_sum - insert_t, insert_t_2, insert_sum_2 - insert_t_2, epsilon);
				vec_res->insert(vec_res->end(), vec2->begin(), vec2->end()); 
				vec_res->push_back(sp_2);
				vector<SegPair*>* vec3 = DauglasPeucker(temps, pos2 + 1, r, insert_t_2, insert_sum_2 - insert_t_2, r_t, r_d, epsilon);
				vec_res->insert(vec_res->end(), vec3->begin(), vec3->end()); 
			}else {
				vector<SegPair*>* vec2 = DauglasPeucker(temps, pos1 + 1, r, insert_t, insert_sum - insert_t, r_t, r_d, epsilon);
				vec_res->insert(vec_res->end(), vec2->begin(), vec2->end()); 
			}
			// delete vec1;
			// delete vec2;
		}
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
		
		for (int i = 0; i < mapmatchPathSet->size(); ++i) {
			
			cout << mapmatchPathSet->at(i) << endl;
			FileReader* mapReader = new FileReader(mapmatchPathSet->at(i), false);
			MapMatchResult* mm = new MapMatchResult(mapReader);
			delete mapReader;
			
        	ofstream* fout_org = new ofstream();
			fout_org->open("origin_DP.txt");
        	ofstream* fout_cprs = new ofstream();
			fout_cprs->open("cprs_DP.txt");

			// Hook: outlier trajectory, directly match to 0
			vector<int>* processedSequence = mm->getProcessedSequence();
			if (!processedSequence->size()) {
				continue;
			}

			// generate temporal component
			for (int j = 1; j < mm->sequence->size(); ++j) {
				if (mm->sequence->at(j)->edgeId == mm->sequence->at(j-1)->edgeId &&
					mm->sequence->at(j)->location < mm->sequence->at(j-1)->location) {
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
			vector<TemporalPair*>* tempPairs = new vector<TemporalPair*>();

			double dis_sofar = 0;
			int obs_cnt = 0; // count the number of observation gps points	
			for (int j = 0; j < mm->sequence->size(); ++j) {
				int len = graph->getEdge(mm->sequence->at(j)->edgeId)->len;
				if (mm->sequence->at(j)->proj_lng > 0){
					dis_sofar = accumulate + len * mm->sequence->at(j)->location;
					tempPairs->push_back(new TemporalPair(mm->sequence->at(j)->t, dis_sofar));
					*fout_org << tempPairs->back()->t<< " "<< tempPairs->back()->d<< endl;
					obs_cnt ++;
				}
				if ((j== mm->sequence->size()-1) ||(mm->sequence->at(j+1)->edgeId != mm->sequence->at(j)->edgeId)){
					spatial->push_back(mm->sequence->at(j)->edgeId);
					accumulate += len;
				}
			}
			PointMatch* header_obs = mm->sequence->at(0);
			PointMatch* tail_obs = mm->sequence->back();
			float header_len = graph->getEdge(header_obs->edgeId)->len * header_obs->location;
			float tail_len = graph->getEdge(tail_obs->edgeId)->len * (1 - tail_obs->location);
			int unit_pos = 1;	
			int last_dis = 0;

			vector<TrajStripe*>* temporal = new vector<TrajStripe*>();
			TrajStripe* trajStripe = new TrajStripe(tempPairs->front(), tempPairs->back());
			temporal->push_back(trajStripe);
			cout<<"GPS size : " << obs_cnt << " .. " << endl;
			int ts_sofar = 0;
			// double travel_sofar = header_len + header_obs->t;
			double Unit_D_T = 10;
			int j = 1;
			double Head = tempPairs->at(0)->t + tempPairs->at(0)->d;
			int Head_t = tempPairs->at(0)->t;
			double Head_d = tempPairs->at(0)->d;
			double travel_sofar = 0;
			int last_ts = 0;
			int size = tempPairs->size();
			for (int _i = 0; _i < size; _i++) {
				tempPairs->at(_i)->t -= Head_t;
				tempPairs->at(_i)->d -= Head_d;
			}
			// *fout_cprs << Head_t <<" "<< Head_d << endl;
			cout << tempPairs->back()->t <<" "<< tempPairs->back()->d << endl;
			trajStripe->stripe = DauglasPeucker(tempPairs, 1, size - 2, 0, 0, tempPairs->back()->t, tempPairs->back()->d, Unit_D_T);

			// while (j < size-1){
			// 	bool justifiable = false; 

			// 	int beta =(int)((tempPairs->at(j)->t + tempPairs->at(j)->d - Head - travel_sofar)/Unit_D_T) + 1;
			// 	int delta = Unit_D_T * (beta + 1);
			// 	while (pave_fitting(delta, j, Head, Head_d, tempPairs, travel_sofar, last_ts, Unit_D_T)){
			// 		beta++;
			// 		delta = Unit_D_T * (beta + 1);
			// 	}
			// 	delta = Unit_D_T * beta;
			// 	int j_at_right = j;
			// 	while (j_at_right < size - 1 && tempPairs->at(j_at_right)->d + tempPairs->at(j_at_right)->t - Head <= delta + travel_sofar) {
			// 		cout << "P " << j_at_right << " "<< tempPairs->at(j_at_right)->t<<" "<< tempPairs->at(j_at_right)->d<< endl;
			// 		j_at_right++;
			// 	} 
			// 	int _t2 = tempPairs->at(j_at_right - 1)->t;
			// 	double _d2 = tempPairs->at(j_at_right - 1)->d;
			// 	int t2 = tempPairs->at(j_at_right)->t;
			// 	double d2 = tempPairs->at(j_at_right)->d;
			// 	int dis_sofar = travel_sofar - last_ts;
			// 	int ts = _t2 + (t2 - _t2)*(delta + travel_sofar + Head - _t2 - _d2 )/(d2+t2 - _t2 - _d2);
			// 	int del_ts = ts -(Head - Head_d) - last_ts;
				
			// 	trajStripe->stripe->push_back(new SegPair(beta, del_ts));
			// 	travel_sofar += delta;
			// 	last_ts += del_ts;
			// 	*fout_cprs << Head_t + last_ts<<" "<< Head_d + travel_sofar - last_ts<< endl;
			// 	cout << "U "<< beta << " " << del_ts << " "<< delta - del_ts << " "<< Head_t + last_ts<<" "<< Head_d + travel_sofar-last_ts<< endl;
			// 	j = j_at_right;
			// }
			trajStripe->display();				
			trajStripe->display(fout_cprs);				
			cout << "End." <<endl;
			// *fout_cprs << tempPairs->back()->t <<" "<< tempPairs->back()->d<< endl;
			fout_org->close();
			fout_cprs->close();
			// RoadNetTrajectory* trajectory = new RoadNetTrajectory(spatial, temporal);
			// trajectory->store(spatialWriter, temporalWriter);
			// delete trajectory;
		}
		
		delete mapmatchPathSet;
		delete spatialWriter;
		delete temporalWriter;
	}
};

PreProcessor* PreProcessor::instance = NULL;

#endif
