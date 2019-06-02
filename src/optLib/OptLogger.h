#pragma once
#include <iostream>
#include <fstream>
using namespace std;
static ofstream * fff = new ofstream;

class OptLogger {

public:

	//Create and open a file stream
	static void begin(std::string _fileLocation) {
		fff->open(_fileLocation);
		//fff->write("asdf", 4);
	}

	//Write an integer
	static void log(int val) {
		*fff << val << endl;
		//std::cout << "Logger: " << val << endl;
	}

	//Write the content of a VectorXd in a CSV format
	static void log(VectorXd val) {
		for (size_t i = 0;  i < val.size()-1;i++)
			*fff << val(i) << ",";
		*fff << val(val.size()-1) << endl;
	}

	//End and close the file stream
	static void end() {
		fff->flush();
		fff->close();
	}
};
