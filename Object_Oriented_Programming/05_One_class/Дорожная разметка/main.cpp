#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "road_marker.hpp"
#include "writer.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Format:\n input.txt output.txt userID percent\n";
        exit(-1);
    }

    std::ifstream ifs(argv[1]);

    if (!ifs)
        return 2;

    std::ofstream ofs(argv[2]);

    try {
        RoadMarker fr;
        Parser parser(ifs);
        Writer writer(ofs);
        fr.markupFiller(parser);
        writer.writeRoadMarking(fr.getCommands());
        std::cout << "Road marking is in " << argv[2] << std::endl;
    }
    catch (std::invalid_argument) {
        std::cout << "Invalid arguments, can't create road marking" << std::endl;
    }
    ifs.close();
    ofs.close();

    return 0;
}
