#include <fstream>

#include "command.hpp"
#include "editor.hpp"
#include "file_io.hpp"
#include "parser.hpp"

int main(int argc, char* argv[]) {
    FileIO fileNames(argv[1], argv[2], argv[3]);
    std::ofstream editedText;
    std::ifstream recipe, textToEdit;

    std::istream& inStream1 = fileNames.setFirstInStream(recipe);
    std::istream& inStream2 = fileNames.setSecondInStream(textToEdit);
    std::ostream& outStream = fileNames.setOutStream(editedText);

    Editor editor = Editor(loadFromStream(inStream1));
    auto Recipe = Command::readRecipe(inStream2);

    editor.applyRecipe(Recipe);

    outStream << editor;
    return 0;
}
