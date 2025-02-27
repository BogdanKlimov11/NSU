#include <fstream>
#include <iostream>
#include <cstdlib>

#include "code_review_decorator.hpp"
#include "line_decorator.hpp"
#include "cpp_decorator.hpp"
#include "html_decorator.hpp"

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cout << "Format: Decorator src.txt out1.txt out2.txt out3.txt\n";
        exit(-1);
    }

    std::ifstream ifs(argv[1], std::ios_base::in);
    if (!ifs) {
        return 2;
    }
    std::istream& is = ifs;
    std::string text{std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>()};

    // HTML
    {
        std::ofstream ofs(argv[2]);
        try {
            HTMLDecorator html(std::shared_ptr<Decorator>(new LineDecorator(nullptr)));
            ofs << html.write(text) << std::endl;
            std::cout << "result is in " << argv[2] << std::endl;
        }
        catch (std::out_of_range) {
            std::cout << "Entered wrong text" << std::endl;
        }
        ifs.close();
        ofs.close();
    }

    // CPPDecorator + HTML
    {
        std::ofstream ofs(argv[3]);
        try {
            HTMLDecorator html(std::shared_ptr<Decorator>(new CPPDecorator(nullptr)));
            ofs << html.write(text) << std::endl;
            std::cout << "result is in " << argv[3] << std::endl;
        }
        catch (std::out_of_range) {
            std::cout << "Entered wrong text" << std::endl;
        }
        ifs.close();
        ofs.close();
    }

    // LineDecorator + CPPDecorator + CodeReviewDecorator + HTML
    {
        std::ofstream ofs(argv[4]);
        try {
            std::shared_ptr<Decorator> line_dec(new LineDecorator(nullptr));
            std::shared_ptr<Decorator> cr_dec(new CPPDecorator(line_dec));
            std::shared_ptr<Decorator> cpp_dec(new CodeReviewDecorator(cr_dec));
            HTMLDecorator dec(cpp_dec);
            ofs << dec.write(text) << std::endl;
            std::cout << "result is in " << argv[4] << std::endl;
        }
        catch (std::out_of_range) {
            std::cout << "Entered wrong text" << std::endl;
        }
        ifs.close();
        ofs.close();
    }

    return 0;
}
