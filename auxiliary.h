
#ifndef __TE_auxiliary__
#define __TE_auxiliary__

#include "file_processor.h"
#include "utility.h"

// MBR class with four boundaries (x1, x2, y1, y2) defining the rectangle region in 2D euclidean space.


class Auxiliary {
public:
	int nodeNumber;
	int edgeNumber;
	int fstNumber;
	
	MBR** fstMBR;				// The MBR of a sub-trajectory
	MBR*** spMBR;				// The MBR of a shortest path (between two nodes)
	MBR** edgeMBR;				// The MBR of an edge
	
	double* fstLen;				// The total road network length of a sub-trajectory
	double** spLen;				// The total road netwoek length of a shortest path (between two nodes)
	
	Auxiliary(Graph* graph, BlockSet* blockSet){
		this->fstNumber = blockSet->blockVec.size();
		this->fstMBR = new MBR*[this->fstNumber];
		for (int i = 0; i < this->fstNumber; i++){
			this->fstMBR[i] = new MBR();
			for (auto it = blockSet->blockVec[i]->begin();
					it != blockSet->blockVec[i]->end(); it++){
				this->fstMBR[i]->merge(graph->getEdge(*it)->mbr);
			}
		}

	}	

	// If any FileReader == NULL, then that auxiliary is not loaded.
	Auxiliary(int nodeNumber, int edgeNumber, int fstNumber, FileReader* fstMBRReader, FileReader* spMBRReader, FileReader* edgeMBRReader, FileReader* fstLenReader, FileReader* spLenReader) {
		this->nodeNumber = nodeNumber;
		this->edgeNumber = edgeNumber;
		this->fstNumber = fstNumber;
		
		if (fstMBRReader != NULL) {
			this->fstMBR = new MBR*[fstNumber];
			for (int i = 0; i < this->nodeNumber; ++i) {
				this->fstMBR[i] = new MBR(fstMBRReader);
			}
		}
		
		if (spMBRReader != NULL) {
			this->spMBR = new MBR**[nodeNumber];
			for (int i = 0; i < this->nodeNumber; ++i) {
				this->spMBR[i] = new MBR*[nodeNumber];
				for (int j = 0; j < this->nodeNumber; ++j) {
					this->spMBR[i][j] = new MBR(spMBRReader);
				}
			}
		}
		
		if (edgeMBRReader != NULL) {
			this->edgeMBR = new MBR*[edgeNumber];
			for (int i = 0; i < this->edgeNumber; ++i) {
				this->edgeMBR[i] = new MBR(edgeMBRReader);
			}
		}
		
		if (fstLenReader != NULL) {
			this->fstLen = new double[fstNumber];
			for (int i = 0; i < this->fstNumber; ++i) {
				this->fstLen[i] = fstLenReader->nextDouble();
			}
		}
		
		if (spLenReader != NULL) {
			this->spLen = new double*[nodeNumber];
			for (int i = 0; i < this->nodeNumber; ++i) {
				this->spLen[i] = new double[nodeNumber];
				for (int j = 0; j < this->nodeNumber; ++j) {
					this->spLen[i][j] = spLenReader->nextDouble();
				}
			}
		}
	}
};

#endif
