#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include "point.hpp"
#include "parser.hpp"

using edge = std::pair<size_t, size_t>; // pair of v1-v2: (Point1->id, Point2->id)

class Graph final {
private:
    std::vector<std::pair<double, edge>> Edges;
    std::vector<Point> Vertices;

public:
    Graph() = default;
    ~Graph() = default;

    void fillGraph(Parser &parser);
    void addNewPoint(const Point &point);
    void createEdges();
    void addWeightedEdge(const Point &v1, const Point &v2);

    size_t edgesSize() const { return Edges.size(); }
    size_t verticesSize() const { return Vertices.size(); }

    const std::vector<std::pair<double, edge>> &getAllEdges() const;
    const std::vector<Point> &getAllPoints() const;
};
