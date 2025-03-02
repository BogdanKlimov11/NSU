#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "budget.hpp"
#include "writer.hpp"

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cout << "Format:\n expected.txt real.txt out.txt dd.mm.yyyy dd.mm.yyyy \n ";
        exit(-1);
    }

    std::ifstream ifs1(argv[1]);
    std::ifstream ifs2(argv[2]);
    std::ofstream ofs(argv[3]);
    if (!ifs1)
        return 2;
    if (!ifs2)
        return 2;

    try {
        Budget b;
        ExpectedSpendingParser parsere(ifs1);
        RealSpendingParser parserr(ifs2);
        Writer writer(ofs);
        Date d1{argv[4]};
        Date d2{argv[5]};
        b.expectedSpendingsFiller(parsere);
        b.realSpendingsFiller(parserr);
        writer.writeSpendings(b.calculate(d1, d2));
        std::cout << "Calculated budget is in " << argv[3] << std::endl;
    } catch (const std::invalid_argument& ex) {
        std::cout << ex.what() << std::endl;
    }

    ifs1.close();
    ifs2.close();
    ofs.close();
    return 0;
}
