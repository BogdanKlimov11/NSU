#include "gtest.h"
#include "disjoint_set.hpp"
#include "graph.hpp"
#include "kruskal.hpp"
#include "parser.hpp"
#include "point.hpp"
#include "writer.hpp"

TEST(GraphTests, Throws) {
    {
        Graph graph;
        EXPECT_EQ(graph.verticesSize(), 0);
        EXPECT_EQ(graph.edgesSize(), 0);
        EXPECT_ANY_THROW(auto result = Kruskal(graph));
    }

    {
        Graph graph;
        Point point(4, 12, -7);
        graph.addNewPoint(point);
        EXPECT_EQ(graph.verticesSize(), 1);
        EXPECT_EQ(graph.edgesSize(), 0);
        EXPECT_ANY_THROW(graph.createEdges());
    }

    {
        Graph graph;
        Point point(4, 12, -7);
        Point point2(4, 12, -7);
        graph.addNewPoint(point);
        graph.addNewPoint(point2);
        EXPECT_EQ(graph.verticesSize(), 1);
        EXPECT_EQ(graph.edgesSize(), 0);
        EXPECT_ANY_THROW(graph.createEdges());
    }
}

TEST(GraphTests, FewPoints) {
    Graph graph;
    Point point1(4, 0, 0);
    Point point2(1, 3, 0);
    Point point3(9, 4, 0);
    Point point4(100, 3, -4);
    graph.addNewPoint(point1);
    graph.addNewPoint(point2);
    graph.addNewPoint(point3);

    EXPECT_NO_THROW(graph.createEdges());

    {
        auto edges = graph.getAllEdges();
        auto points = graph.getAllPoints();
        EXPECT_EQ(graph.verticesSize(), 3);
        EXPECT_EQ(graph.edgesSize(), 3);
        EXPECT_EQ(edges.size(), 3);
        EXPECT_EQ(points.size(), 3);
    }

    graph.addNewPoint(point4);
    graph.addWeightedEdge(point2, point4);
    graph.addWeightedEdge(point1, point4);

    EXPECT_NO_THROW(graph.createEdges());
    auto result = Kruskal(graph);
    EXPECT_EQ(result.size(), 3);
    EXPECT_EQ(result[0].second.first, 1);
    EXPECT_EQ(result[0].second.second, 9);
    EXPECT_EQ(result[0].first, 1);

    EXPECT_EQ(result[1].second.first, 4);
    EXPECT_EQ(result[1].second.second, 1);
    EXPECT_EQ(result[1].first, 3);

    EXPECT_EQ(result[2].second.first, 1);
    EXPECT_EQ(result[2].second.second, 100);
    EXPECT_EQ(result[2].first, 4);
}

TEST(Kruskal, Complex) {
    std::stringstream ss;
    ss << "4, 0, 0" << "\n"
       << "1, 3, 0" << "\n"
       << "9, 4, 0" << "\n"
       << "100, 3, -4" << "\n";
    Parser parser(ss);
    Graph graph;
    graph.fillGraph(parser);
    graph.createEdges();
    EXPECT_EQ(graph.verticesSize(), 4);
    EXPECT_EQ(graph.edgesSize(), 6);
    auto result = Kruskal(graph);
    EXPECT_EQ(result.size(), 3);

    std::stringstream oss;
    std::string s;
    Writer mstWriter(oss);
    mstWriter.writeMSTVector(result);
    std::getline(oss, s);
    EXPECT_EQ(s, "1 - 9 : 1");
    std::getline(oss, s);
    EXPECT_EQ(s, "4 - 1 : 3");
    std::getline(oss, s);
    EXPECT_EQ(s, "1 - 100 : 4");
}

TEST(PointTest, FewPoints) {
    Point p0;
    EXPECT_EQ(p0.getID(), 0);
    EXPECT_EQ(p0.getX(), 0);
    EXPECT_EQ(p0.getY(), 0);

    Point p(10, 2, -78);
    EXPECT_EQ(p.getID(), 10);
    EXPECT_EQ(p.getX(), 2);
    EXPECT_EQ(p.getY(), -78);

    {
        std::stringstream ss;
        std::string s;
        ss << p;
        std::getline(ss, s);
        EXPECT_EQ(s, "10: 2, -78");
    }

    EXPECT_FALSE(p == p0);
    Point p2(10, 2, -78);
    EXPECT_TRUE(p == p2);
}

TEST(DisjointSetTest, Throws) {
    {
        EXPECT_ANY_THROW(DisjointSet ds(0));
    }

    {
        DisjointSet ds(3);
        EXPECT_ANY_THROW(ds.find(7));
        EXPECT_ANY_THROW(ds.setUnion(7, 1));
        EXPECT_ANY_THROW(ds.setUnion(2, 4));
    }
}

TEST(DisjointSetTest, Find) {
    DisjointSet ds(3);
    EXPECT_EQ(ds.find(0), 0);
    EXPECT_EQ(ds.find(1), 1);
    EXPECT_EQ(ds.find(2), 2);
    EXPECT_ANY_THROW(ds.find(4));
}

TEST(DisjointSetTest, Union) {
    DisjointSet ds(3);
    EXPECT_NO_THROW(ds.setUnion(1, 2));
    EXPECT_EQ(ds.find(1), 2);
    EXPECT_EQ(ds.find(2), 2);
    EXPECT_ANY_THROW(ds.find(7));

    EXPECT_NO_THROW(ds.setUnion(0, 1));
    EXPECT_EQ(ds.find(0), 2);
    EXPECT_EQ(ds.find(1), 2);
    EXPECT_EQ(ds.find(2), 2);
    EXPECT_ANY_THROW(ds.find(4));
}

TEST(DisjointSetTest, Indexer) {
    {
        std::vector<size_t> v = {4, 0, 7, 3, 8};
        EXPECT_EQ(indexer(v, 4), 0);
        EXPECT_EQ(indexer(v, 8), 4);
        EXPECT_EQ(indexer(v, 3), 3);
    }

    {
        std::vector<size_t> v = {2, 1};
        EXPECT_EQ(indexer(v, 3), -1);
    }

    {
        std::vector<size_t> v;
        EXPECT_ANY_THROW(indexer(v, 3));
    }
}

TEST(DisjointSetTest, Complex) {
    DisjointSet ds(4);
    std::vector<size_t> idxr;
    for (size_t i = 0; i < 4; i++) {
        idxr.push_back(i);
    }
    EXPECT_EQ(indexer(idxr, 2), 2);
    EXPECT_EQ(indexer(idxr, 4), -1);
    auto v1 = ds.find(indexer(idxr, 1));
    auto v2 = ds.find(indexer(idxr, 2));
    EXPECT_EQ(v1, 1);
    EXPECT_EQ(v2, 2);
    if (v1 != v2) {
        ds.setUnion(v1, v2);
    }
    EXPECT_EQ(ds.find(indexer(idxr, 1)), 2);
    EXPECT_EQ(ds.find(indexer(idxr, 2)), 2);
}

TEST(Streams, ParserRight) {
    std::istringstream ss{
        "4, 0, 0 \n"
        "1, 3, 0 \n"
        "9, 4, 0 \n"
        "100, 3, -4 \n"};
    Parser parser(ss);

    auto point1 = parser.getNextPoint();
    EXPECT_EQ(point1.first.getID(), 4);
    EXPECT_EQ(point1.first.getX(), 0);
    EXPECT_EQ(point1.first.getY(), 0);
    EXPECT_TRUE(point1.second);

    auto expect1 = std::make_pair<Point, bool>({4, 0, 0}, true);
    auto expect2 = std::make_pair<Point, bool>({1, 3, 0}, true);
    auto expect3 = std::make_pair<Point, bool>({9, 4, 0}, true);
    auto expect4 = std::make_pair<Point, bool>({100, 3, -4}, true);

    auto point2 = parser.getNextPoint();
    auto point3 = parser.getNextPoint();
    auto point4 = parser.getNextPoint();

    EXPECT_EQ(point1, expect1);
    EXPECT_EQ(point2, expect2);
    EXPECT_EQ(point3, expect3);
    EXPECT_EQ(point4, expect4);
}

TEST(Streams, ParserInvalid) {
    std::istringstream ss{
        "4, 0, fdfa, eq \n"
        "1, \n"
        "9, 4, 0 \n"
        "\n"
        "word\n"
        "100, afafaf 3, uii -4 \n"
        "1, 2, 3 \n"};
    Parser parser(ss);
    auto emptyPoint = std::make_pair<Point, bool>({0, 0, 0}, false);

    auto point1 = parser.getNextPoint();
    EXPECT_EQ(point1.first.getID(), 9);
    EXPECT_EQ(point1.first.getX(), 4);
    EXPECT_EQ(point1.first.getY(), 0);
    EXPECT_TRUE(point1.second);

    auto point2 = parser.getNextPoint();
    EXPECT_EQ(point2.first.getID(), 1);
    EXPECT_EQ(point2.first.getX(), 2);
    EXPECT_EQ(point2.first.getY(), 3);
    EXPECT_TRUE(point2.second);

    EXPECT_EQ(parser.getNextPoint(), emptyPoint);
}

TEST(Streams, WriterRight) {
    std::vector<std::pair<double, edge>> result = {
        {1.0, {1, 9}},
        {3.0, {4, 1}}};

    std::ostringstream oss;
    Writer mstWriter(oss);
    mstWriter.writeMSTVector(result);
    auto expect =
        "1 - 9 : 1\n"
        "4 - 1 : 3\n";

    EXPECT_EQ(oss.str(), expect);
}

TEST(Streams, WriterEmpty) {
    std::vector<std::pair<double, edge>> result;
    std::ostringstream oss;
    Writer mstWriter(oss);
    mstWriter.writeMSTVector(result);
    EXPECT_EQ(oss.str(), "");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
