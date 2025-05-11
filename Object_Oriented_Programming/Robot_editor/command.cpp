#include "command.hpp"
#include "parser.hpp"
#include "utilities.hpp"

std::vector<Command> Command::readRecipe(std::istream& is) {
    std::vector<Command> recipe;
    std::string line;
    while (std::getline(is, line)) {
        auto words = createWordList(line);
        Command command;
        if (parseDelete(words, command) or parseReplace(words, command) or parseChange(words, command) or
            parseInsert(words, command) or parseUndo(words, command))
            recipe.push_back(command);
    }
    return recipe;
}

possibleCommands Command::getCmd() const { return cmd; }
unsigned long Command::getStart() const { return start; }
unsigned long Command::getFinish() const { return finish; }
std::string Command::getText1() const { return text1; }
std::string Command::getText2() const { return text2; }

void Command::setCmd(const possibleCommands& _cmd) { cmd = _cmd; }
void Command::setStart(unsigned long _start) { start = _start; }
void Command::setFinish(unsigned long _finish) { finish = _finish; }
void Command::setText1(const std::string& _text1) { text1 = _text1; }
void Command::setText2(const std::string& _text2) { text2 = _text2; }
