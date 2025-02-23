#include <string>
#include <algorithm>
#include <stdexcept>

#include "graph.hpp"

void Graph::fillGraph(Parser &parser) {
    while (true) {
        std::pair<Point, bool> newPoint = parser.getNextPoint();
        if (!newPoint.second) break;
        addNewPoint(newPoint.first);
    }
}

void Graph::addNewPoint(const Point &point) {
    if (std::find(Vertices.begin(), Vertices.end(), point) == Vertices.end()) {
        Vertices.push_back(point);
    }
}

void Graph::createEdges() {
    /* Creates all Edges to strongly connected graph */
    if (Vertices.size() <= 1) {
        throw std::length_error("Vertices size is incorrect");
    }
    
    std::vector<Point>::iterator i_itr, j_itr;
    for (i_itr = Vertices.begin(); i_itr < Vertices.end() - 1; i_itr++) {
        for (j_itr = i_itr + 1; j_itr < Vertices.end(); j_itr++) {
            addWeightedEdge(*i_itr, *j_itr);
        }
    }
}

void Graph::addWeightedEdge(const Point &v1, const Point &v2) {
    /* For each 2 points in Vertices vector we get weight
       and add new edge into the Edges vector */
    double w = sqrt(pow(v1.getX() - v2.getX(), 2) + pow(v1.getY() - v2.getY(), 2));
    auto edgePair = std::make_pair(w, std::make_pair(v1.getID(), v2.getID()));
    
    if (w != 0.0) {
        Edges.push_back(edgePair);
    }
}

const std::vector<std::pair<double, edge>>& Graph::getAllEdges() const {
    return Edges;
}

const std::vector<Point>& Graph::getAllPoints() const {
    return Vertices;
}
