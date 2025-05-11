#include <cassert>
#include <string>
#include <istream>
#include <sstream>

#include "editor.hpp"

namespace {
    std::vector<std::string> split(const std::string& text) {
        std::istringstream ss(text);
        std::string buf;
        std::vector<std::string> v;
        while (std::getline(ss, buf, '\n')) {
            v.push_back(buf);
        }
        return v;
    }
}

void Editor::deleteLines(unsigned long start, unsigned long finish) {
    const size_t size = paragraphs.size();
    if (start <= 0 || start > size || start > finish)
        return;
    backupParagraphs = paragraphs;
    const auto start_it = paragraphs.begin() + start - 1;
    const auto finish_it = finish > size ? paragraphs.end() : paragraphs.begin() + finish;
    auto count = finish_it - start_it;
    paragraphs.erase(start_it, finish_it);
}

void Editor::insertLines(unsigned long pos, const std::string& additionalText) {
    if (pos > paragraphs.size() || pos < 0)
        return;
    backupParagraphs = paragraphs;
    std::vector<std::string> additionalParagraphs = split(additionalText);
    paragraphs.insert(paragraphs.begin() + pos, additionalParagraphs.begin(), additionalParagraphs.end());
}

void Editor::changeLines(unsigned long start, unsigned long finish, const std::string& newText) {
    const size_t size = paragraphs.size();
    if (start <= 0 || start > size || start > finish)
        return;
    backupParagraphs = paragraphs;
    deleteLines(start, finish);
    std::vector<std::string> backup = backupParagraphs;
    insertLines(start - 1, newText);
    backupParagraphs = backup;
}

void Editor::replaceLines(unsigned long start, unsigned long finish, const std::string& oldText, const std::string& newText) {
    const size_t size = paragraphs.size();
    if (start <= 0 || start > size || start > finish)
        return;
    backupParagraphs = paragraphs;
    size_t changes_count = 0, offset = 0;
    const auto start_it = paragraphs.begin() + start - 1;
    const auto finish_it = finish > size ? paragraphs.end() : paragraphs.begin() + finish;
    std::string changeBuf;
    for (auto it = start_it; it != finish_it; ++it) {
        changeBuf += (*it);
        changeBuf += '\n';
    }
    for (auto pos = changeBuf.find(oldText, offset); pos != std::string::npos; pos = changeBuf.find(oldText, offset)) {
        changeBuf.replace(pos, oldText.size(), newText);
        ++changes_count;
        offset = pos + newText.size();
    }
    auto changeParagraphs = split(changeBuf);
    std::copy(changeParagraphs.begin(), changeParagraphs.end(), start_it);
}

void Editor::undo() {
    paragraphs = backupParagraphs;
}

void Editor::applyRecipe(const std::vector<Command>& receipt) {
    for (auto& command : receipt) {
        auto cmd = command.getCmd();
        switch (cmd) {
            case possibleCommands::DELETE:
                deleteLines(command.getStart(), command.getFinish());
                break;
            case possibleCommands::INSERT:
                insertLines(command.getStart(), command.getText1());
                break;
            case possibleCommands::CHANGE:
                changeLines(command.getStart(), command.getFinish(), command.getText1());
                break;
            case possibleCommands::REPLACE:
                replaceLines(command.getStart(), command.getFinish(), command.getText1(), command.getText2());
                break;
            case possibleCommands::UNDO:
                undo();
                break;
            case possibleCommands::EMPTY:
                break;
        }
    }
}

std::ostream& operator<<(std::ostream& out, const Editor& editor) {
    for (auto& line : editor.paragraphs)
        out << line << std::endl;
    return out;
}
