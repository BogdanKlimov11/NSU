#include "kruskal.hpp"

std::vector<std::pair<double, edge>> Kruskal(const Graph & graph) {
    /* The Kruskal's algorithm sorts edges by weight
       then checks if the cycle is created with current edge.
       i.e. if 2 vertices of this edge belong to the same parent */
    if (graph.verticesSize() == 0) {
        throw std::invalid_argument("Size of the graph is incorrect!");
    }
    if (graph.edgesSize() == 0) {
        throw std::invalid_argument("Number of graph's edges is incorrect!");
    }

    std::vector<std::pair<double, edge>> result;
    std::vector<std::pair<double, edge>> Edges = graph.getAllEdges();
    std::vector<Point> Vertices = graph.getAllPoints();

    std::sort(Edges.begin(), Edges.end(), 
        [](const std::pair<double, edge> &left, const std::pair<double, edge> &right) {
            return left.first < right.first; 
        });

    std::vector<std::pair<double, edge>>::iterator it = std::unique(Edges.begin(), Edges.end());
    Edges.resize(std::distance(Edges.begin(), it));

    DisjointSet DSTree(graph.verticesSize()); 
    std::vector<size_t> idxr;
    for (auto point : Vertices) {
        idxr.push_back(point.getID());
    }

    std::vector<std::pair<double, edge>>::iterator edge_itr;
    for (edge_itr = Edges.begin(); edge_itr != Edges.end(); edge_itr++) {
        size_t v1 = edge_itr->second.first;
        size_t v2 = edge_itr->second.second;
        double w = edge_itr->first;

        size_t setV1 = DSTree.find(indexer(idxr, v1));
        size_t setV2 = DSTree.find(indexer(idxr, v2));

        // Checking if the cycle was created:
        if (setV1 != setV2) {
            result.push_back(std::make_pair(w, std::make_pair(v1, v2)));
            DSTree.setUnion(setV1, setV2);
        }
    }
    return result;
}

size_t indexer(std::vector<size_t> & idxr, size_t id) {
    if (idxr.empty()) {
        throw std::out_of_range("Empty idxr vector");
    }

    for (size_t i = 0; i < idxr.size(); i++) {
        if (idxr[i] == id) return i;
    }
    return -1;
}
