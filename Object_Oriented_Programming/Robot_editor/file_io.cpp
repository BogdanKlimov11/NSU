#include <fstream>

#include "file_io.hpp"

std::ostream& FileIO::setOutStream(std::ofstream& fout) const {
    fout.open(getOutputName());
    std::ostream& outStream = (getOutputName().empty()) ? std::cout : fout;
    return outStream;
}

std::istream& FileIO::setFirstInStream(std::ifstream& fin) const {
    fin.open(getFirstInputName());
    std::istream& inStream1 = (getFirstInputName().empty()) ? std::cin : fin;
    return inStream1;
}

std::istream& FileIO::setSecondInStream(std::ifstream& fin) const {
    fin.open(getSecondInputName());
    std::istream& inStream2 = (getSecondInputName().empty()) ? std::cin : fin;
    return inStream2;
}
