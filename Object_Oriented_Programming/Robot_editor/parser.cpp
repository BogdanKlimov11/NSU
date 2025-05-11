#include "parser.hpp"
#include "utilities.hpp"

using namespace std;

vector<std::string> loadFromStream(istream& input) {
    vector<std::string> lines;
    string line;
    while (getline(input, line))
        lines.push_back(line);
    return lines;
}

bool parseDelete(const std::vector<std::string>& words, Command& command) {
    const size_t size = words.size();
    if (size == 0)
        return false;
    if (words[0] != "delete")
        return false;

    command.setCmd(possibleCommands::DELETE);
    command.setStart(1);
    command.setFinish(Command::END);

    auto rest = size - 1;
    auto index = 1;

    if (rest > 2 && words[index] == "from" && convertToInt(words[index + 1])) {
        command.setStart(convertToInt(words[index + 1]));
        index += 2;
        rest -= 2;
    }

    if (rest == 2 && words[index] == "to" && convertToInt(words[index + 1])) {
        command.setFinish(convertToInt(words[index + 1]));
        rest -= 2;
    }

    return rest == 0;
}

bool parseUndo(const std::vector<std::string>& words, Command& command) {
    const size_t size = words.size();
    if (size == 0)
        return false;
    if (words[0] != "undo")
        return false;

    if (size == 1) {
        command.setCmd(possibleCommands::UNDO);
        return true;
    }
    return false;
}

bool parseInsert(const std::vector<std::string>& words, Command& command) {
    const size_t size = words.size();
    if (size == 0)
        return false;
    if (words[0] != "insert")
        return false;

    command.setCmd(possibleCommands::INSERT);
    command.setStart(1);
    command.setFinish(Command::END);

    auto rest = size - 1;
    auto index = 1;

    if (rest > 2 && words[index] == "after" && convertToInt(words[index + 1])) {
        command.setStart(convertToInt(words[index + 1]));
        index += 2;
        rest -= 2;
    }

    if (rest == 1 && inQuotes(words[index])) {
        auto text = words[index];
        deleteQuotes(text);
        command.setText1(text);
        rest -= 1;
    }

    return rest == 0;
}

bool parseChange(const std::vector<std::string>& words, Command& command) {
    const size_t size = words.size();
    if (size == 0)
        return false;
    if (words[0] != "change")
        return false;

    command.setCmd(possibleCommands::CHANGE);
    command.setStart(1);
    command.setFinish(Command::END);

    auto rest = size - 1;
    auto index = 1;

    if (rest > 2 && words[index] == "from" && convertToInt(words[index + 1])) {
        command.setStart(convertToInt(words[index + 1]));
        index += 2;
        rest -= 2;
    }

    if (rest > 2 && words[index] == "to" && convertToInt(words[index + 1])) {
        command.setFinish(convertToInt(words[index + 1]));
        index += 2;
        rest -= 2;
    }

    if (rest == 2 && words[index] == "with" && inQuotes(words[index + 1])) {
        auto text1 = words[index + 1];
        deleteQuotes(text1);
        command.setText1(text1);
        rest -= 2;
    }

    return rest == 0;
}

bool parseReplace(const std::vector<std::string>& words, Command& command) {
    const size_t size = words.size();
    if (size == 0)
        return false;
    if (words[0] != "replace")
        return false;

    command.setCmd(possibleCommands::REPLACE);
    command.setStart(1);
    command.setFinish(Command::END);

    auto rest = size - 1;
    auto index = 1;

    if (rest > 2 && words[index] == "from" && convertToInt(words[index + 1])) {
        command.setStart(convertToInt(words[index + 1]));
        index += 2;
        rest -= 2;
    }

    if (rest > 2 && words[index] == "to" && convertToInt(words[index + 1])) {
        command.setFinish(convertToInt(words[index + 1]));
        index += 2;
        rest -= 2;
    }

    if (rest > 1 && inQuotes(words[index])) {
        auto text1 = words[index];
        deleteQuotes(text1);
        command.setText1(text1);
        index += 1;
        rest -= 1;
    }

    if (rest == 2 && words[index] == "with" && inQuotes(words[index + 1])) {
        auto text2 = words[index + 1];
        deleteQuotes(text2);
        command.setText2(text2);
        rest -= 2;
    }

    return rest == 0;
}
