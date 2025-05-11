#include <sstream>
#include <iostream>
#include <ctime>

#include "parser.hpp"
#include "writer.hpp"
#include "frequency_dictionary.hpp"
#include "map_dictionary.hpp"
#include "vector_dictionary.hpp"
#include "list_dictionary.hpp"

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cout << "Format:\nFrequencyDictionary src.txt map.txt list.txt vector.txt\n";
        exit(-1);
    }

    {
        std::ifstream ifs(argv[1]);
        if (!ifs)
            return 2;
        std::ofstream ofs(argv[2]);
        auto start = clock();
        try {
            MapDictionary dict;
            Parser fileParser(ifs);
            fileParser.fillDictionary(&dict);
            writeDictionary(ofs, &dict);
            std::cout << "Dict is in " << argv[2] << std::endl;
        }
        catch (std::out_of_range) {
            std::cout << "Entered wrong text" << std::endl;
        }
        auto time = clock() - start;
        ifs.close();
        ofs.close();
        std::cout << " time:" << (double)time / CLOCKS_PER_SEC << "sec" << std::endl;
    }

    {
        std::ifstream ifs(argv[1]);
        if (!ifs)
            return 2;
        std::ofstream ofs(argv[3]);
        auto start = clock();
        try {
            ListDictionary dict;
            Parser fileParser(ifs);
            fileParser.fillDictionary(&dict);
            writeDictionary(ofs, &dict);
            std::cout << "Dict is in " << argv[3] << std::endl;
        }
        catch (std::out_of_range) {
            std::cout << "Entered wrong text" << std::endl;
        }
        auto time = clock() - start;
        ifs.close();
        ofs.close();
        std::cout << " time:" << (double)time / CLOCKS_PER_SEC << "sec" << std::endl;
    }

    {
        std::ifstream ifs(argv[1]);
        if (!ifs)
            return 2;
        std::ofstream ofs(argv[4]);
        auto start = clock();
        try {
            VectorDictionary dict;
            Parser fileParser(ifs);
            fileParser.fillDictionary(&dict);
            writeDictionary(ofs, &dict);
            std::cout << "Dict is in " << argv[4] << std::endl;
        }
        catch (std::out_of_range) {
            std::cout << "Entered wrong text" << std::endl;
        }
        auto time = clock() - start;
        ifs.close();
        ofs.close();
        std::cout << " time:" << (double)time / CLOCKS_PER_SEC << "sec" << std::endl;
    }

    return 0;
}
