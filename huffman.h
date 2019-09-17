

#ifndef __TE_huffman__
#define __TE_huffman__

#include <iostream>
#include <vector>
#include <algorithm>

// #include "road_trajectory.h"
#include "config.h"

using namespace std;

typedef pair<int, int> IntPair;

// Huffman Tree Structure Node
class HuffmanNode {
public:
	int id;
	int weight;
	int father;
	int leftSon;
	int rightSon;
	
	HuffmanNode(int id, int weight, int leftSon, int rightSon) {
		this->id = id;
		this->weight = weight;
		this->leftSon = leftSon;
		this->rightSon = rightSon;
	}
	
	void display() {
		cout << id << " weight: " << weight
		<< "\t father: " << father << "\t leftSon: " << leftSon << "\t rightSon: " << rightSon << endl;
	}
};
// Leaf Huffman Node has block tag
class HuffmanLeafNode: public HuffmanNode {
public:
	int blockId;
	
	HuffmanLeafNode(int id, int blockId, int weight, int leftSon, int rightSon): HuffmanNode(id, weight, leftSon, rightSon) {
		this->blockId = blockId;
	}
	
	void display() {
		cout << id << " weight: " << weight
		<< "\t father: " << father << "\t leftSon: " << leftSon << "\t rightSon: " << rightSon
		<< "\t block: " << blockId << endl;
	}
};

// Huffman Code Binary Code;
class HuffmanCode {
public:
	vector<bool> code;
};

// Huffman Tree Structure
class HuffmanTree {
private:
	// recursively display Huffman Tree
	void display(HuffmanNode* node) {
		if (node->leftSon == Config::NULL_POINTER && node->rightSon == Config::NULL_POINTER) {
			((HuffmanLeafNode*)node)->display();
		} else {
			node->display();
		}
		if (node->leftSon != Config::NULL_POINTER) {
			display(getNode(node->leftSon));
		}
		if (node->rightSon != Config::NULL_POINTER) {
			display(getNode(node->rightSon));
		}
	}
	
	vector<bool> code;	// huffman code buffer
	// assign huffman code
	void assignHuffmanCode(int blockId) {
		this->blockHuffmanCode[blockId] = new HuffmanCode();
		for (int i = 0; i < code.size(); ++i) {
			this->blockHuffmanCode[blockId]->code.push_back(code[i]);
		}
	}
	// recursively get huffman code
	void getHuffmanCode(HuffmanNode* node) {
		if (node->leftSon == Config::NULL_POINTER && node->rightSon == Config::NULL_POINTER) {
			assignHuffmanCode(((HuffmanLeafNode*)node)->blockId);
			return;
		}
		code.push_back(false);
		getHuffmanCode(getNode(node->leftSon));
		code.pop_back();
		
		code.push_back(true);
		getHuffmanCode(getNode(node->rightSon));
		code.pop_back();
	}
	
public:
	int huffmanSize, blockSize;
	vector<HuffmanNode*>* huffman = new vector<HuffmanNode*>();
	map<int, HuffmanCode*> blockHuffmanCode;	// each block node's huffman code
	int root;
	
	HuffmanTree(map<int, int>* freq) {
		if (freq == NULL) {
			throw "freq NULL exception";
		}
		this->blockSize = freq->size();
		// blockHuffmanCode = new HuffmanCode*[this->blockSize];		// the huffman code of each block node
		IntPair* blockList = new IntPair[this->blockSize];
        int i = 0;
		for (auto it = freq->begin(); it != freq->end(); ++it) {
			// cout<< it->second << ' ' << it->first << endl;
			// if (it->first == 4990)
			// 	cout<< it->second << ' ' << it->first << endl;
			blockList[i++] = IntPair(it->second, it->first);
		}
		
		sort(blockList, blockList + this->blockSize);
	    FileWriter* bLfw = new FileWriter("freq.txt", false);

        for(int i = 0; i < this->blockSize; ++i){
            bLfw->writeInt(blockList[i].first);
            bLfw->writeChar('\n');
        }
        
		vector<int> hfmBranchNode;
		
		int ptFirst = 0, ptSecond = 0;
		huffmanSize = 0;
		cout << "total: " << this->blockSize << endl;
		for (int i = 0; i < this->blockSize; ++i) {
			// if (i % 10000 == 0) {
			// 	cout << i << "\t";
			// }
			
			int minFrequency = Config::HUGE_NUMBER;
			int operation = 0;
			if (ptFirst < this->blockSize - 1 && minFrequency > blockList[ptFirst].first + blockList[ptFirst + 1].first) {
				minFrequency = blockList[ptFirst].first + blockList[ptFirst + 1].first;
				operation = 1;
			}

		if (ptSecond < (int)hfmBranchNode.size() - 1 && minFrequency > getNode(hfmBranchNode[ptSecond])->weight + getNode(hfmBranchNode[ptSecond + 1])->weight) {
			minFrequency = getNode(hfmBranchNode[ptSecond])->weight + getNode(hfmBranchNode[ptSecond + 1])->weight;
			operation = 2;
		}
		if (ptFirst < this->blockSize && ptSecond < (int)hfmBranchNode.size() && minFrequency > blockList[ptFirst].first+ getNode(hfmBranchNode[ptSecond])->weight) {
			minFrequency = blockList[ptFirst].first + getNode(hfmBranchNode[ptSecond])->weight;
			operation = 3;
		}
		// if (blockList[ptFirst].second == 4990) cout << operation << endl;
		switch (operation) {
			case 1: {
				HuffmanLeafNode* hfmNode1 = new HuffmanLeafNode(huffmanSize, blockList[ptFirst].second, blockList[ptFirst].first, Config::NULL_POINTER, Config::NULL_POINTER);
				HuffmanLeafNode* hfmNode2 = new HuffmanLeafNode(huffmanSize + 1, blockList[ptFirst + 1].second, blockList[ptFirst + 1].first, Config::NULL_POINTER, Config::NULL_POINTER);
				HuffmanNode* hfmNode3 = new HuffmanNode(huffmanSize + 2, minFrequency, hfmNode1->id, hfmNode2->id);
				hfmNode1->father = hfmNode3->id;
				hfmNode2->father = hfmNode3->id;
				hfmBranchNode.push_back(hfmNode3->id);
				huffman->push_back(hfmNode1);
				huffman->push_back(hfmNode2);
				huffman->push_back(hfmNode3);
				huffmanSize += 3;
				ptFirst += 2;
				break;
			}
			case 2: {
				HuffmanNode* hfmNode3 = new HuffmanNode(huffmanSize, minFrequency, hfmBranchNode[ptSecond], hfmBranchNode[ptSecond + 1]);
				getNode(hfmBranchNode[ptSecond])->father = hfmNode3->id;
				getNode(hfmBranchNode[ptSecond + 1])->father = hfmNode3->id;
				hfmBranchNode.push_back(hfmNode3->id);
				huffman->push_back(hfmNode3);
				huffmanSize ++;
				ptSecond += 2;
				break;
			}
			case 3: {
				HuffmanLeafNode* hfmNode1 = new HuffmanLeafNode(huffmanSize, blockList[ptFirst].second, blockList[ptFirst].first, Config::NULL_POINTER, Config::NULL_POINTER);
				HuffmanNode* hfmNode2 = getNode(hfmBranchNode[ptSecond]);
				HuffmanNode* hfmNode3 = new HuffmanNode(huffmanSize + 1, minFrequency, hfmNode1->id, hfmNode2->id);
				hfmNode1->father = hfmNode3->id;
				hfmNode2->father = hfmNode3->id;
				hfmBranchNode.push_back(hfmNode3->id);
				huffman->push_back(hfmNode1);
				huffman->push_back(hfmNode3);
				huffmanSize += 2;
				ptFirst ++;
				ptSecond ++;
			}
			default: {
				this->root = getNode(huffmanSize - 1)->id;
			}
		}
	}
	
	cout << endl;
	
	hfmBranchNode.clear();
	
	// get huffman code
	getHuffmanCode(getNode(root));
}

HuffmanTree(FileReader* fr) {
	huffmanSize = fr->nextInt();
	blockSize = fr->nextInt();
	root = fr->nextInt();
	for (int i = 0; i < huffmanSize; ++i) {
		int id = fr->nextInt();
		int weight = fr->nextInt();
		int father = fr->nextInt();
		int leftSon = fr->nextInt();
		int rightSon = fr->nextInt();
		int blockId = fr->nextInt();
		HuffmanNode* node;
		if (blockId == Config::NULL_POINTER) {
			node = new HuffmanNode(id, weight, leftSon, rightSon);
		} else {
			node = new HuffmanLeafNode(id, blockId, weight, leftSon, rightSon);
		}
		node->father = father;
		huffman->push_back(node);
	}
	
	// blockHuffmanCode = new HuffmanCode*[blockSize];		// the huffman code of each block node
	
	// get huffman code
	getHuffmanCode(getNode(root));
}

HuffmanNode* getNode(int id) {
	if (this->huffman->at(id)->id == id) {
		return this->huffman->at(id);
		}
		for (int i = 0; i < this->huffman->size(); ++i) {
			if (this->huffman->at(i)->id == id) {
				return this->huffman->at(i);
			}
		}
		return NULL;
	}
	
	// store the Huffman tree structure
	void store(FileWriter* fw) {
		fw->writeInt(huffmanSize);
		fw->writeInt(blockSize);
		fw->writeInt(root);
		for (int i = 0; i < huffman->size(); ++i) {
			fw->writeInt(huffman->at(i)->id);
			fw->writeInt(huffman->at(i)->weight);
			fw->writeInt(huffman->at(i)->father);
			fw->writeInt(huffman->at(i)->leftSon);
			fw->writeInt(huffman->at(i)->rightSon);
			
			if (huffman->at(i)->leftSon == Config::NULL_POINTER && huffman->at(i)->rightSon == Config::NULL_POINTER) {
				fw->writeInt(((HuffmanLeafNode*)huffman->at(i))->blockId);
			} else {
				fw->writeInt(Config::NULL_POINTER);
			}
		}
	}
	
	void display() {
		display(getNode(root));
	}
	
	void displayCode() {
		for (int i = 0; i < this->blockSize; ++i) {
			cout << "block " << i << ": ";
			for (int j = 0; j < blockHuffmanCode[i]->code.size(); ++j) {
				cout << blockHuffmanCode[i]->code[j];
			}
			cout << endl;
		}
	}
	
	~HuffmanTree() {
		for (auto it = blockHuffmanCode.begin(); it != blockHuffmanCode.end(); it++)
			delete it->second;
		for (auto it = huffman->begin(); it != huffman->end(); it++)
			delete *it;
		delete huffman;
	}
};

#endif
