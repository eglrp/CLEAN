
#ifndef __TE_Pattern_temporal_cprs_h
#define __TE_Pattern_temporal_cprs_h

#include "utility.h"
#include "file_processor.h"
#include "road_network.h"
#include "road_trajectory.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include<iomanip>

using namespace std;


class TemporalCPRS {
private:
	static TemporalCPRS* instance;
	
public:
	static TemporalCPRS* getInstance() {
		if (instance == NULL) {
			instance = new TemporalCPRS();
		}
		return instance;
	}
	
	vector<SegPair*>* DauglasPeucker(vector<TemporalPairInt*>* temps, int l, int r, int l_t, double l_d, int r_t, double r_d, double epsilon){
		int pos = -1;
		double score = -9999;
		bool enough = true;
		double max_err = 999999;
		vector<SegPair*>* vec_res = new vector<SegPair*>();
		// cout << l << " "<< r << " "<< " "<<endl;
		if (l > r) return vec_res;
		double sum_l = l_t + l_d;
		double sum_r = r_t + r_d;
		if (sum_r - sum_l <= epsilon) return vec_res;
		for (int i = l; i <= r; i++){
			int t = temps->at(i)->t;
			double d = temps->at(i)->d;
			double interpo_d = l_d + (r_d - l_d) * (t - l_t) / (r_t - l_t);
			double interpo_t = l_t + (r_t - l_t) * (d - l_d) / (r_d - l_d);
			double err_d = abs(d - interpo_d);
			double err_t = abs(t - interpo_t);
			// if (err_d > max_err) err_d = 0;
			// if (err_t > max_err) err_t = 0;
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
			int atLeft = 1;
			if (sum_r <= (beta+1) * epsilon || (sum_l < beta * epsilon && abs(beta * epsilon - sum_dt) < abs((beta+1) * epsilon - sum_dt))){
				insert_sum = beta * epsilon;
				int pre_pos = pos;
				while (true){		
					sum_dt = temps->at(pre_pos)->d + temps->at(pre_pos)->t;
					sum_dt_pre = temps->at(pre_pos - 1)->d + temps->at(pre_pos - 1)->t;
					if (sum_dt_pre <= insert_sum && sum_dt >= insert_sum){
						break;
					}
					pre_pos--;
				}
				insert_t = (int)(temps->at(pre_pos)->t - (temps->at(pre_pos)->t - temps->at(pre_pos - 1)->t)*(sum_dt - insert_sum)/(sum_dt - sum_dt_pre));	
			} else {
				insert_sum = (beta + 1) * epsilon;
				beta++;
				int aft_pos = pos;
				while (true){		
					sum_dt = temps->at(aft_pos)->d + temps->at(aft_pos)->t;
					sum_dt_aft = temps->at(aft_pos + 1)->d + temps->at(aft_pos + 1)->t;
					if (sum_dt <= insert_sum && sum_dt_aft >= insert_sum){
						break;
					}
					aft_pos++;
				}
				// cout << aft_pos  << " " << sum_dt << " " << insert_sum << endl;
				// cout << temps->at(aft_pos)->d << " " << temps->at(aft_pos)->t << endl;
				// cout << l_d << " " << l_t << " " << sum_l<< endl;
				// cout << r_d << " " << r_t << " " << sum_r <<endl;
				insert_t = (int)(temps->at(aft_pos)->t + (temps->at(aft_pos + 1)->t - temps->at(aft_pos)->t)*(insert_sum - sum_dt)/(sum_dt_aft- sum_dt));	
				// cout<< "====" <<endl;
				// cout <<setiosflags(ios::fixed);
				// cout<< temps->at(aft_pos + 1)->t << " " << temps->at(aft_pos)->t<< endl;
				// cout << insert_sum  << " " << sum_dt << " " << sum_dt_aft << endl;
				// cout <<temps->at(aft_pos + 1)->t - temps->at(aft_pos)->t <<endl;
				// cout << setprecision(4) << insert_sum  << " - " << sum_dt <<" = "<<insert_sum - sum_dt << endl;
				// cout <<sum_dt_aft- sum_dt  << endl;
				// cout <<(insert_sum - sum_dt)/(sum_dt_aft- sum_dt) << endl;

				// cout <<(temps->at(aft_pos + 1)->t - temps->at(aft_pos)->t)*(insert_sum - sum_dt)/(sum_dt_aft- sum_dt)<< endl;
			
				// cout << insert_t<< " " << insert_sum - insert_t << endl;
				atLeft = 0;
			}

			// cout << score << " " << atLeft << endl;
			SegPair* sp = new SegPair(beta, insert_t);
			vector<SegPair*>* vec1 = DauglasPeucker(temps, l, pos - atLeft, l_t, l_d, insert_t, insert_sum - insert_t, epsilon);
			vector<SegPair*>* vec2 = DauglasPeucker(temps, pos + !atLeft, r, insert_t, insert_sum - insert_t, r_t, r_d, epsilon);
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

	void compress(char* spatialPath, char* temporalPath, char* tempCprsPath, int num, int W, int Err) {
		
		FileReader* spatial = new FileReader(spatialPath, true);
		FileReader* temporal = new FileReader(temporalPath, true);
		FileWriter* tempCprs = new FileWriter(tempCprsPath, true);	
		int traNumber = spatial->nextInt();
		if (num < traNumber) traNumber = num;
		temporal->nextInt();
		int origin_volume = 0;
		int cprs_volume =0;
		cout << traNumber << endl;
		for (int i = 0; i < traNumber; ++i) {
			RoadNetTrajectory* rnt = new RoadNetTrajectory(spatial, temporal);
			TrajStripe* trajStripe = new TrajStripe(rnt->temporal->front(), rnt->temporal->back());
			double Unit_D_T = 10;
			int Head_t = rnt->temporal->at(0)->t;
			double Head_d = rnt->temporal->at(0)->d;
            //if (Head_t <=0 || Head_d <= 0) continue;
			// cout << rnt->temporal->size() << endl;
			vector<TemporalPairInt*>* tempPairs = new vector<TemporalPairInt*>();
			cout <<setiosflags(ios::fixed);
			for (int _i = 0; _i < rnt->temporal->size(); _i++) {
                int x = rnt->temporal->at(_i)->t - Head_t;
                int y = (int)(rnt->temporal->at(_i)->d - Head_d);
                //if (x <= 0 || y <= 0 || x <= tempPairs->back()->t || y <= tempPairs->back()->d) continue;
				tempPairs->push_back(new TemporalPairInt(x,y));
				//cout << setprecision(4) << _i << " " << tempPairs->back()->t << " " <<tempPairs->back()->d << endl; 
			}
            //if (tempPairs->size() < 3) continue;
			
			trajStripe->stripe = DauglasPeucker(tempPairs, 1, tempPairs->size() - 2, 0, 0, tempPairs->back()->t, tempPairs->back()->d, Err);
			// cprs_volume += TrajStripe->stripe->size();
			//cout << rnt->temporal->size() << " "<< trajStripe->stripe->size() << endl;
			trajStripe->store(tempCprs, W, Err);
			delete rnt;
			delete trajStripe;
		}
		// cout << origin_volume << " " << cprs_volume << endl;		
		delete spatial;
		delete temporal;
		delete tempCprs;
	}
};

TemporalCPRS* TemporalCPRS::instance = NULL;

#endif
