#include "..\include\Utils.h"


std::string readValidLine(std::ifstream& stream) {
	std::string line;
	while (std::getline(stream, line) && (line[0] == '#' || line[0] == '\0')) {
	}
	return line;
}
