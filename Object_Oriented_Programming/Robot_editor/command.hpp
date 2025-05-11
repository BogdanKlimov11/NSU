#ifndef LAB_REDACTOR_COMMAND_HPP
#define LAB_REDACTOR_COMMAND_HPP

#include <string>
#include <vector>

enum class possibleCommands {
    DELETE,
    INSERT,
    CHANGE,
    REPLACE,
    UNDO,
    EMPTY
};

class Command final {
    possibleCommands cmd;
    unsigned long start, finish;
    std::string text1, text2;

public:
    explicit Command(const possibleCommands& _cmd = possibleCommands::EMPTY, unsigned long _start = 1,
                     unsigned long _finish = END, const std::string& _text1 = "", const std::string& _text2 = "") :
        cmd(_cmd), start(_start), finish(_finish), text1(_text1), text2(_text2) {}

    static std::vector<Command> readRecipe(std::istream& is);
    static const unsigned long END = std::numeric_limits<unsigned long>::max();

    [[nodiscard]] possibleCommands getCmd() const;
    [[nodiscard]] unsigned long getStart() const;
    [[nodiscard]] unsigned long getFinish() const;
    [[nodiscard]] std::string getText1() const;
    [[nodiscard]] std::string getText2() const;

    void setCmd(const possibleCommands& _cmd);
    void setStart(unsigned long _start);
    void setFinish(unsigned long _finish);
    void setText1(const std::string& _text1);
    void setText2(const std::string& _text2);
};

#endif
