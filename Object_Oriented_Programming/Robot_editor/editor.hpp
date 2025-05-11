#ifndef LAB_REDACTOR_EDITOR_HPP
#define LAB_REDACTOR_EDITOR_HPP

#include <string>

#include "command.hpp"

class Editor final {
    std::vector<std::string> paragraphs;
    std::vector<std::string> backupParagraphs;

public:
    Editor() = default;
    explicit Editor(const std::vector<std::string>& _paragraphs) : paragraphs(_paragraphs), backupParagraphs(_paragraphs) {}

    void deleteLines(unsigned long start, unsigned long finish);
    void changeLines(unsigned long start, unsigned long finish, const std::string& newText);
    void insertLines(unsigned long pos, const std::string& additionalText);
    void replaceLines(unsigned long start, unsigned long finish, const std::string& oldText, const std::string& newText);
    void undo();

    void applyRecipe(const std::vector<Command>& receipt);
    friend std::ostream& operator<<(std::ostream& out, const Editor& editor);
};

#endif
