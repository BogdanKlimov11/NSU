#include "road_marker.hpp"

namespace {
    CommandType switch_condition(CommandType cond) {
        return (cond == up) ? down : up;
    }
}

void RoadMarker::SwitchAdd(double lenght) {
    current_command = switch_condition(current_command);
    Result.push_back(Command{ current_coord, current_command });
    current_coord += lenght;
}

void RoadMarker::addCommand(double streak, double gap, double lenght) {
    double expected_coord = current_coord + lenght;
    int n;
    if (streak / gap == 1.0) {
        n = static_cast<int>(lenght * 2);
    }
    else {
        n = static_cast<int>(lenght / 2);
    }

    for (int i = 1; i <= n; i++) {
        switch (current_command) {
        case up:
            SwitchAdd(streak);
            break;
        case down:
            SwitchAdd(gap);
            break;
        };
    }
    if (expected_coord - current_coord != 0.0) {
        SwitchAdd(expected_coord - current_coord);
    }
}

void RoadMarker::createLine(const Line & line) {
    switch (line.type) {
    case continuous:
        if (current_command == up) {
            SwitchAdd(line.lenght);
        }
        else {
            current_coord += line.lenght;
        }
        break;
    case broken:
        addCommand(1.0, 3.0, line.lenght);
        break;
    case changing:
        addCommand(3.0, 1.0, line.lenght);
        break;
    case stripes:
        addCommand(0.5, 0.5, line.lenght);
        break;
    case empty:
        if (current_command == down) {
            SwitchAdd(line.lenght);
        }
        else {
            current_coord += line.lenght;
        }
        break;
    }
}

void RoadMarker::markupFiller(Parser & parser) {
    auto lines = parser.getLines();
    if (!lines.second) return;

    Lines = lines.first;
}

const std::vector<Command>& RoadMarker::getCommands() {
    if (Lines.empty()) {
        throw std::invalid_argument("Markups is empty!");
    }
    current_coord = 0.0;
    current_command = up;
    for (auto line : Lines) {
        createLine(line);
    }
    Result.push_back(Command{ current_coord, up }); // means end
    return Result;
}
