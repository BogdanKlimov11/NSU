#ifndef LAB_REDACTOR_FILEIO_HPP
#define LAB_REDACTOR_FILEIO_HPP

#include <iostream>
#include <string>
#include <vector>

class FileIO final {
private:
    std::string TextFileName;
    std::string Recipe;
    std::string OutputFileName;

public:
    FileIO(const std::string& textFileName, const std::string& recipe, const std::string& outputFileName)
        : TextFileName(textFileName), Recipe(recipe), OutputFileName(outputFileName) {}

    [[nodiscard]] inline std::string getFirstInputName() const { return TextFileName; }
    [[nodiscard]] inline std::string getSecondInputName() const { return Recipe; }
    [[nodiscard]] inline std::string getOutputName() const { return OutputFileName; }

    std::istream& setFirstInStream(std::ifstream& fin) const;
    std::istream& setSecondInStream(std::ifstream& fin) const;
    std::ostream& setOutStream(std::ofstream& fout) const;
};

#endif
