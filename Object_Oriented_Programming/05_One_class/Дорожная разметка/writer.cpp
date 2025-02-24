#include "writer.hpp"

namespace {
    std::string toString(CommandType type) {
        return type == up ? "up" : "down";
    }
}

void Writer::writeRoadMarking(const std::vector<Command>& commands) {
    if (commands.empty()) return;
    for (size_t i = 0; i < commands.size(); i++) {
        os << commands[i].coord << " ";
        os << toString(commands[i].type) << '\n';
    }
}
