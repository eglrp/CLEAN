#ifndef __TE_MBR__
#define __TE_MBR__

#include <iostream>

#include "file_processor.h"

using namespace std;

// MBR class with four boundaries (x1, x2, y1, y2) defining the rectangle region in 2D euclidean space.
class MBR {
public:
	double x1, x2, y1, y2;		// MBR boundary
	MBR() {
		this->x1 = 0; this->x2 = 0;
		this->y1 = 0; this->y2 = 0;
	}
	
	// Construct MBR from boundary params
	MBR(double x1, double x2, double y1, double y2) {
		this->x1 = x1; this->x2 = x2;
		this->y1 = y1; this->y2 = y2;
	}
	
	// load MBR from file
	MBR(FileReader* fr) {
		this->x1 = fr->nextDouble();
		this->x2 = fr->nextDouble();
		this->y1 = fr->nextDouble();
		this->y2 = fr->nextDouble();
	}

    void merge(EcldPoint* point){
        if (this->x1 == 0 || point->x < this->x1) this->x1 = point->x;
        if (this->x2 == 0 || point->x > this->x2) this->x2 = point->x;
        if (this->y1 == 0 || point->y < this->y1) this->y1 = point->y;
        if (this->y2 == 0 || point->y > this->y2) this->y2 = point->y;
    }

    void merge(MBR* other){
        if (this->x1 == 0 || other->x1 < this->x1) this->x1 = other->x1; 
        if (this->x2 == 0 || other->x2 > this->x2) this->x2 = other->x2; 
        if (this->y1 == 0 || other->y1 < this->y1) this->y1 = other->y1;
        if (this->y2 == 0 || other->y2 > this->y2) this->y2 = other->y2;
    }
	
	// Store MBR to file
	void store(FileWriter* fw) {
		fw->writeDouble(this->x1);
		fw->writeDouble(this->x2);
		fw->writeDouble(this->y1);
		fw->writeDouble(this->y2);
	}
	
	// Judge if two MBR has intersection with each other
	bool intersect(MBR* other) {
		return (this->x1 < other->x2 && this->x2 > other->x1 && this->y1 < other->y2 && this->y2 > other->y1);
	}
	
	// Judge if a EcldPoint locates inside the MBR
	bool contain(EcldPoint* point) {
		return point->x >= this->x1 && point->x <=this->x2 && point->y >= this->y1 && point->y <= this->y2;
	}
	
	// Judge if a line crosses the MBR
	bool cross(EcldPoint* p1, EcldPoint* p2) {
		EcldPoint* b1 = new EcldPoint(x1, y1);
		EcldPoint* b2 = new EcldPoint(x2, y2);
		EcldPoint* b3 = new EcldPoint(x1, y2);
		EcldPoint* b4 = new EcldPoint(x2, y1);
		bool result = vectorIntersect(p1, p2, b1, b2) || vectorIntersect(p1, p2, b3, b4);
		delete b1;
		delete b2;
		delete b3;
		delete b4;
		return result;
	}
};

#endif