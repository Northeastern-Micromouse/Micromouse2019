#include "tableReader.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace micromouse {

TableReader::TableReader(std::string fpath) : fpath(fpath) {}

std::vector<float>* TableReader::parseTableCSV() {
    std::ifstream csvFile(this->fpath);
    auto table = new std::vector<float>();

    // populate header information
    std::string header;
    std::getline(csvFile, header);
    std::stringstream headerStream(header);
    std::string token;
    std::getline(headerStream, token, ',');
    this->brightness = std::stoi(token);

    while (csvFile) {
        std::string line;
        std::getline(csvFile, line);

        std::stringstream lineStream(line);
        std::string entry;
        while(std::getline(lineStream, entry, ',')) {
            if (entry.empty()) {
                continue;
            }
            table->push_back(std::stof(entry));
        }
    }
    return table;
}

/*
int TableReader::getMinValue() {
    return this->minValue;
}
*/
int TableReader::getBrightness() {
    return this->brightness;
}

} // namespace micromouse
