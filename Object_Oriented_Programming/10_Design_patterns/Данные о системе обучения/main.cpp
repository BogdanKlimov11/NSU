#include <fstream>

#include "data_base.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Format:\n input.txt output.tsv\n";
        return -1;
    }

    Factory f;
    std::ifstream ifs(argv[1]);
    if (!ifs) {
        return 2;
    }

    std::ofstream ofs(argv[2]);

    try {
        DataBaseFiller filler(ifs, f);
        DataBase db;
        db.fill(filler);
        db.serialize(ofs);
        std::cout << "Result in " << argv[2] << std::endl;
    } catch (const std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
