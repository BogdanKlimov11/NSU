#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "parser.hpp"
#include "friend_recommendation.hpp"
#include "writer.hpp"

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cout << "Format:\n input.txt output.txt userID percent\n";
        exit(-1);
    }

    std::ifstream ifs(argv[1]);

    if (!ifs)
        return 2;

    std::ofstream ofs(argv[2]);    
    FriendRecommendation fr;
    Parser parser(ifs);
    Writer writer(ofs);
    try {
        fr.friendFiller(parser);
        writer.writeRecommendedFriends(fr.RecommendTo(argv[3], std::stoi(argv[4])));
        std::cout << "Recommended friends is in " << argv[2] << std::endl;
    }
    catch (std::invalid_argument) {
        std::cout << "Invalid arguments, can't recommend friends" << std::endl;
    }
    ifs.close();
    ofs.close();

    return 0;
}
