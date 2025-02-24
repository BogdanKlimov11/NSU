#pragma once

enum LineType {
    continuous,
    broken,
    changing,
    stripes,
    empty
};

enum CommandType { up, down };

struct Line {
    double lenght;
    LineType type;
    bool operator==(const Line& other) const {
        return (lenght == other.lenght) && (type == other.type);
    }
};

struct Command {
    double coord;
    CommandType type;
    bool operator==(const Command& other) const {
        return (coord == other.coord) && (type == other.type);
    }
};
