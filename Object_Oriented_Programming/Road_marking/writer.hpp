#pragma once

#include <iostream>
#include <vector>
#include <sstream>
#include <string>

#include "line.h"

class Writer {
private:
    std::ostream& os;

public:
    Writer(std::ostream& _os) : os(_os) {}
    void writeRoadMarking(const std::vector<Command>& commands);
};
