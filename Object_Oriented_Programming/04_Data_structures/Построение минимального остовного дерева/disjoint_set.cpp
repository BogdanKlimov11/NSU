#include "disjoint_set.hpp"

DisjointSet::DisjointSet(size_t n) {
    if (n == 0) {
        throw std::invalid_argument("Number of Vertices is incorrect!");
    }
    size = n;

    for (size_t i = 0; i <= size; i++) {
        parent.emplace_back(i);
        rank.emplace_back(0);
    }
}

size_t DisjointSet::find(size_t u) {
    /* If u is the parent of itself, then return u.
       Else u is not the representative of it set,
       so we recursively call find on its parent */

    if (u > size) {
        throw std::out_of_range("incorrect id in function find!");
    }
    if (u != parent[u]) {
        return DisjointSet::find(parent[u]);
    } else {
        return u;
    }
}

void DisjointSet::setUnion(size_t v1, size_t v2) {
    /* Make a subtree of the other tree */
    if ((v1 > size) || (v2 > size)) {
        throw std::out_of_range("incorrect id to set union!");
    }
    v1 = DisjointSet::find(v1);
    v2 = DisjointSet::find(v2);

    if (rank[v1] > rank[v2]) {
        parent[v2] = v1;
    } else if (rank[v1] <= rank[v2]) {
        parent[v1] = v2;
    }

    if (rank[v1] == rank[v2]) {
        rank[v2] = rank[v2] + 1;
    }
}
