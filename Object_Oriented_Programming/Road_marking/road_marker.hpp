#pragma once

#include <vector>
#include <math.h>

#include "parser.h"

class RoadMarker {
private:
    std::vector<Line> Lines;
    std::vector<Command> Result;
    double current_coord = 0.0;
    CommandType current_command = up;

    void SwitchAdd(double lenght);
    void addCommand(double streak, double gap, double lenght);
    void createLine(const Line& line);
public:
    RoadMarker() = default;
    void markupFiller(Parser & parser);
    const std::vector<Command>& getCommands();
};
