#ifndef __TE_file_processor__
#define __TE_file_processor__

#include <iostream>
#include <vector>
#include <cstdlib>
#include <dirent.h>
#include <cstring>
#include <exception>

using namespace std;

//convertor between char and binary char
union Char2Binary {
	char value;
	unsigned char data[1];
};
union Char2Binary char2Binary;

//convertor between short and binary short
union Short2Binary {
	short value;
	unsigned char data[2];
};
union Short2Binary short2Binary;

//convertor between short and binary short
union UShort2Binary {
	unsigned short value;
	unsigned char data[2];
};
union UShort2Binary ushort2Binary;

//convertor between int and binary int
union Int2Binary {
	int value;
	unsigned char data[4];
};
union Int2Binary int2Binary;

//convertor between double and binary double
union Double2Binary {
	double value;
	unsigned char data[8];
};
union Double2Binary double2Binary;

//File Reader
class FileReader {
	FILE* fp;
	bool binary;

public:
	FileReader(char* fileName, bool binary) {
		this->binary = binary;
		fp = fopen(fileName, binary ? "rb" : "r");
        if (!fp) {
            cout << fileName << endl;
            throw invalid_argument("file not found");
        }
    }

	FileReader(char* fileName, char* options, bool binary) {
		this->binary = binary;
		fp = fopen(fileName, options);
	}

	//read a char
	char nextChar() {
		if (this->binary) {
			if (fread(char2Binary.data,sizeof(unsigned char), 1,fp) < 1) return EOF;
			return char2Binary.value;
		} else {
			char temp;
			if (fscanf(fp, "%c", &temp) == EOF) {
				return EOF;
			}
			return temp;
		}
	}

	//read a short int
	short nextShort() {
		if (this->binary) {
			if (fread(short2Binary.data,sizeof(unsigned char), 2,fp) < 2) return EOF;
			return short2Binary.value;
		} else {
			short temp;
			if (fscanf(fp, "%hd", &temp) == EOF) {
				return EOF;
			}
			return temp;
		}
	}
	
	//read a unsigned short int
	unsigned short nextUShort() {
		if (this->binary) {
			if (fread(ushort2Binary.data,sizeof(unsigned char), 2,fp) < 2) return EOF;
			return ushort2Binary.value;
		} else {
			unsigned short temp;
			if (fscanf(fp, "%hd", &temp) == EOF) {
				return EOF;
			}
			return temp;
		}
	}

	//read an int
	int nextInt() {
		if (this->binary) {
			if (fread(int2Binary.data,sizeof(unsigned char), 4,fp) < 4) return EOF;
			return int2Binary.value;
		} else {
			int temp;
			int sig = fscanf(fp, "%d", &temp);
			if ( sig == EOF || sig == 0) {
				return EOF;
			}
			return temp;
		}
	}

	//read a double
	double nextDouble() {
		if (this->binary) {
			if (fread(double2Binary.data,sizeof(unsigned char), 8,fp) < 8) return EOF;
			return double2Binary.value;
		} else {
			double temp;
			if (fscanf(fp, "%lf", &temp) == EOF) {
				return EOF;
			}
			return temp;
		}
	}

	// read a string (shorter than 256)
	char* nextString() {
		char* temp = new char[256];
		if (this->binary) {
			int i = 0;
			while (fread(char2Binary.data,sizeof(unsigned char), 1,fp) > 0) {
				temp[i] = (char)char2Binary.value;
				if (temp[i++] == 0 || i > 254) {
					break;
				}
			}
			if (i == 0) {
				return NULL;
			}
			temp[i] = 0;
			return temp;
		} else {
			if (fscanf(fp, "%s", temp) == EOF) {
				return NULL;
			}
			return temp;
		}
	}

	bool isBinary() {
		return this->binary;
	}

	~FileReader() {
		fclose(fp);
	}
};

//File Writer
class FileWriter {
	FILE* fp;
	bool binary;

public:
	FileWriter(char* fileName,  bool binary) {
		this->binary = binary;
		fp = fopen(fileName, binary ? "wb" : "w");
	}

	FileWriter(char* fileName, char* options, bool binary) {
		this->binary = binary;
		fp = fopen(fileName, options);
	}

	//write a char
	void writeChar(char value) {
		if (this->binary) {
			char2Binary.value = value;
			fwrite(char2Binary.data,sizeof(unsigned char), 1,fp);
		} else {
			fprintf(fp, "%c", value);
		}
	}

	//write a short int
	void writeShort(short value) {
		if (this->binary) {
			short2Binary.value = value;
			fwrite(short2Binary.data,sizeof(unsigned char), 2,fp);
		} else {
			fprintf(fp, "%hd", value);
		}
	}

	//write an int
	void writeInt(int value) {
		if (this->binary) {
			int2Binary.value = value;
			fwrite(int2Binary.data,sizeof(unsigned char), 4,fp);
		} else {
			fprintf(fp, "%d", value);
		}
	}

	//write an ushort 
	void writeUShort(unsigned short value) {
		if (this->binary) {
			ushort2Binary.value = value;
			fwrite(ushort2Binary.data,sizeof(unsigned char), 2,fp);
		} else {
			fprintf(fp, "%d", value);
		}
	}

	//write a double
	void writeDouble(double value) {
		if (this->binary) {
			double2Binary.value = value;
			fwrite(double2Binary.data,sizeof(unsigned char), 8,fp);
		} else {
			fprintf(fp, "%lf", value);
		}
	}

	bool isBinary() {
		return this->binary;
	}

	~FileWriter() {
		fclose(fp);
	}
};

// Binary bit stream
class Binary {
private:
	int byteNumber(int number) {
		return number / 8 + (number % 8 ? 1 : 0);
	}
	
public:
	int number;
	vector<bool>* binary;
	
	Binary(vector<bool>* binary) {
		this->binary = binary;
		this->number = (int)binary->size();
	}
	
	Binary(FileReader* fr) {
		this->number = fr->nextInt();
		int byteNumber = this->byteNumber(this->number);
		this->binary = new vector<bool>();
		for (int i = 1; i < byteNumber; ++i) {
			unsigned char byte = fr->nextChar();
			int mask = 0x80;
			for (int i = 0; i < 8; ++i) {
				binary->push_back(byte & mask);
				mask >>= 1;
			}
		}
		unsigned char byte = fr->nextChar();
		if (this->number % 8) {
			int mask = 1 << this->number % 8;
			for (int i = 0; i < this->number % 8; ++i) {
				mask >>= 1;
				binary->push_back(byte & mask);
			}
		} else {
			int mask = 0x80;
			for (int i = 0; i < 8; ++i) {
				binary->push_back(byte & mask);
				mask >>= 1;
			}
		}
	}
	
	void store(FileWriter* fw) {
		fw->writeInt(this->number);
        if (this->number == 0) return;
		int byteNumber = this->byteNumber(this->number);
		for (int i = 0; i < byteNumber - 1; ++i) {
			int byte = 0;
			for (int j = 0; j < 8; ++j) {
				byte = byte * 2 + (this->binary->at((i << 3) + j) ? 1 : 0);
			}
			fw->writeChar((unsigned char) byte);
		}
		int i = byteNumber - 1;
		if (this->number % 8) {
			int byte = 0;
			for (int j = 0; j < this->number % 8; ++j) {
				byte = byte * 2 + (this->binary->at((i << 3) + j) ? 1 : 0);
			}
			fw->writeChar((unsigned char) byte);
		} else {
			int byte = 0;
			for (int j = 0; j < 8; ++j) {
				byte = byte * 2 + (this->binary->at((i << 3) + j) ? 1 : 0);
			}
			fw->writeChar((unsigned char) byte);
		}
	}
	
	void display() {
		cout << this->number << endl;
		for (int i = 0; i < this->number; ++i) {
			cout << binary->at(i) << " ";
		}
		cout << endl;
	}
	
	~Binary() {
		delete binary;
	}
};

struct ExprConfig {
    char* dirpath;
    int K;
    int trajsNumber;
    char* algo;
    int W;
    int Err;
    char* getFilename(char* ext){
        string dirpath = this->dirpath;
        string strK = "K" + to_string(this->K);
        string strW = "W" + to_string(this->W);
        string strErr = "Err" + to_string(this->Err);
        string strDB = to_string(this->trajsNumber);
		string outpath;
		if (ext == "")
			outpath = dirpath + algo + "_" + strK + "_" + strW +"_"+strErr + "_" + strDB;
		else
			outpath = dirpath + algo + "_" + strK + "_" + strW +"_"+strErr + "_" + strDB + "." + ext;
        // cout << outpath << endl;
        char* cstr = new char[outpath.length()+1];
        strcpy(cstr, outpath.c_str()); 
        return cstr;
    }
     
};

struct ExprResult{
    char* exprName;
    char* getFilename(char* extension){
		char* filename = new char[128];
		memset(filename, 0, 128);
        strcat(filename, exprName); 
		strcat(filename, ".");
		strcat(filename, extension);
		return filename;
	}
};

// Other tool functions
class FileTool {
private:
	static FileTool* instance;
	
public:
	static FileTool* getInstance() {
		if (instance == NULL) {
			instance = new FileTool();
		}
		return instance;
	}
	
	// get path filename set (filepath length should not exceed 128)
	vector<char*>* getFileNameSet(char* path) {
		vector<char*>* result = new vector<char*>();
		struct dirent* ent = NULL;
		DIR *dir;
		dir = opendir(path);
		while((ent = readdir(dir))!= NULL){
			if (ent->d_name[0]=='.') continue;
			char* name = new char[128];
			memset(name, 0, 128);
			strcat(name, path);
			strcat(name, "/");
			strcat(name, ent->d_name);
			result->push_back(name);
		}
		closedir(dir);
		return result;
	}

	vector<char*>* getExprFiles(char* path){
        vector<char* >* fileNameSet = getFileNameSet(path);
        vector<char* >* result  = new vector<char* >;
        for (int i = 0; i < fileNameSet->size(); i++){
            char* filename = fileNameSet->at(i);
            int filename_len = strlen(fileNameSet->at(i)); 
            if (filename_len >= 4 && strcmp(filename + filename_len - 4, ".ptr") == 0) {
                filename[filename_len-4]=0;
                result->push_back(filename);
            }
        }
        return result;
    }
};

FileTool* FileTool::instance = NULL;

#endif // __TE_file_processor__
