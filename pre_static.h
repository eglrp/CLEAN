
#ifndef PRESS_v2_pre_processing_h
#define PRESS_v2_pre_processing_h

#include "utility.h"
#include "file_processor.h"
#include "road_network.h"
#include "road_trajectory.h"
#include <vector>

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

			// Hook: outlier trajectory, directly match to 0
			vector<int>* processedSequence = mm->getProcessedSequence();
			if (!processedSequence->size()) {
				continue;
				processedSequence->push_back(0);
				mm->sequence->at(0)->edgeId = 0;
			}

			// generate temporal component
			for (int j = 1; j < mm->sequence->size(); ++j) {
				if (mm->sequence->at(j)->edgeId == Config::NULL_POINTER) {
					mm->sequence->at(j)->edgeId = mm->sequence->at(j - 1)->edgeId;
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
			int obs_cnt = 0; // count the number of observation gps points	
			for (int j = 0; j < mm->sequence->size(); ++j) {
				int len = graph->getEdge(mm->sequence->at(j)->edgeId)->len;
				if (mm->sequence->at(j)->proj_lng > 0){
					obs_cnt ++;
				}
				if ((j==0)||(spatial->back() != mm->sequence->at(j)->edgeId)){
					spatial->push_back(mm->sequence->at(j)->edgeId);
					accumulate += len;
				}
			}
			PointMatch* header_obs = mm->sequence->at(0);
			PointMatch* tail_obs = mm->sequence->back();
			float header_len = graph->getEdge(header_obs->edgeId)->len * header_obs->location;
			float tail_len = graph->getEdge(tail_obs->edgeId)->len * (1 - tail_obs->location);
			accumulate = accumulate - header_len - tail_len;
			double unit_len = accumulate / (obs_cnt - 1);

			int unit_pos = 1;	
			double travel_sofar = - header_len;
			int last_dis = 0;
			int last_ts = header_obs->t;
			int dis_sofar = 0;
			int ts_sofar = 0;
			vector<int> * tmp_spa = new vector<int>();
			vector<int> * temporal = new vector<int>();
			temporal->push_back(last_ts);
			int tmp_cnt = 1;
			cout<<"GPS size : " << obs_cnt << " Unit_length: "<< unit_len << endl;
			for (int j = 1; j < mm->sequence->size()-1; ++j) {
				int len = graph->getEdge(mm->sequence->at(j)->edgeId)->len;
				if (mm->sequence->at(j)->proj_lng > 0){
					dis_sofar = travel_sofar + len * mm->sequence->at(j)->location;
					ts_sofar = mm->sequence->at(j)->t;
					tmp_cnt ++;
					if (last_dis < dis_sofar){
						while (unit_pos * unit_len <= dis_sofar){
							int unit_ts = (int)(last_ts + (ts_sofar - last_ts)*(unit_pos * unit_len - last_dis) / (dis_sofar - last_dis));
							cout << "unit " <<unit_pos << " : delta timestamp = " << unit_ts - temporal->back() << endl;
							temporal->push_back(unit_ts);
							unit_pos ++;
						}
					}
					cout << "P " << tmp_cnt <<" delta dis = "<< dis_sofar -last_dis << " delta ts="<< ts_sofar - last_ts <<endl;
					last_dis = dis_sofar;
					last_ts = ts_sofar;	
				}
				if (mm->sequence->at(j+1)->edgeId != mm->sequence->at(j)->edgeId){
					travel_sofar += len;
				}
			}
			cout << tail_obs->t - temporal->back() << endl;
			cout << "End." <<endl;
			temporal->push_back(tail_obs->t);

			RoadNetTrajectory* trajectory = new RoadNetTrajectory(spatial, temporal);
			trajectory->store(spatialWriter, temporalWriter);
			delete trajectory;
		}
		
		delete mapmatchPathSet;
		delete spatialWriter;
		delete temporalWriter;
	}
};

PreProcessor* PreProcessor::instance = NULL;

#endif
