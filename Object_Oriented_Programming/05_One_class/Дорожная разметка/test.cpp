#include "gtest/gtest.h"
#include "road_marker.hpp"
#include "writer.hpp"

TEST(Parser, GetLineRight) {
    std::istringstream ss{
        "20 continuous\n"
        "20 broken\n"
        "10 changing\n"
        "20 empty\n"
        "3 stripes\n" };
    Parser parser(ss);

    Line expect1{20, continuous};
    Line expect2{20, broken};
    Line expect3{10, changing};
    Line expect4{20, empty};
    Line expect5{3, stripes};

    auto lines = parser.getLines();

    EXPECT_EQ(lines.first[0], expect1);
    EXPECT_EQ(lines.first[1], expect2);
    EXPECT_EQ(lines.first[2], expect3);
    EXPECT_EQ(lines.first[3], expect4);
    EXPECT_EQ(lines.first[4], expect5);

    EXPECT_EQ(lines.second, true);
}

TEST(Parser, GetLineInvalid1) {
    std::istringstream ss{
        "20 contin\n"
        ".././/dfa\n"
        "empty 30\n"
        "10 changing\n" };
    Parser parser(ss);
    EXPECT_ANY_THROW(parser.getLines());
}

TEST(Parser, GetLineInvalid2) {
    std::istringstream ss{
        "20 contin\n"
        ".././/dfa\n"
        "empty 30\n" };
    Parser parser(ss);
    EXPECT_ANY_THROW(parser.getLines());
}

TEST(Parser, GetLineInvalid3) {
    std::istringstream ss{"bom changing\n"};
    Parser parser(ss);
    auto lines = parser.getLines();
    EXPECT_EQ(lines.second, false);
    EXPECT_EQ(lines.first.size(), 0);
}

TEST(Parser, Convert) {
    EXPECT_EQ(Parser::convert("continuous"), continuous);
    EXPECT_EQ(Parser::convert("broken"), broken);
    EXPECT_EQ(Parser::convert("changing"), changing);
    EXPECT_EQ(Parser::convert("empty"), empty);
    EXPECT_EQ(Parser::convert("stripes"), stripes);
    EXPECT_ANY_THROW(Parser::convert("other"));
}

TEST(RoadMarker, Complex1) {
    std::istringstream ss{
        "10 continuous\n"
        "4 broken\n"
        "4 changing\n"
        "10 continuous\n" };
    std::vector<Command> expected = {
        Command{0.0, down}, Command{10.0, up}, Command{13.0, down}, Command{14.0, up},
        Command{15.0, down}, Command{28.0, up}
    };
    Parser parser(ss);
    RoadMarker marker;
    marker.markupFiller(parser);

    const auto& result = marker.getCommands();
    EXPECT_EQ(result, expected);
}

TEST(RoadMarker, Complex2) {
    std::istringstream ss{
        "1 empty\n"
        "7 broken\n"
        "9 changing\n"
        "10 empty\n" };
    std::vector<Command> expected = {
        Command{1.0, down}, Command{2.0, up}, Command{5.0, down}, Command{6.0, up},
        Command{8.0, down}, Command{11.0, up}, Command{12.0, down}, Command{15.0, up},
        Command{16.0, down}, Command{17.0, up}, Command{27.0, up}
    };
    Parser parser(ss);
    RoadMarker marker;
    marker.markupFiller(parser);

    const auto& result = marker.getCommands();
    EXPECT_EQ(result, expected);
}

TEST(RoadMarker, Complex3) {
    std::istringstream ss{
        "3 broken\n"
        "2.7 stripes\n"
        "10 continuous\n" };
    std::vector<Command> expected = {
        Command{0.0, down}, Command{1.0, up}, Command{3.0, down}, Command{3.5, up},
        Command{4.0, down}, Command{4.5, up}, Command{5.0, down}, Command{5.5, up},
        Command{5.7, down}, Command{15.7, up}
    };
    Parser parser(ss);
    RoadMarker marker;
    marker.markupFiller(parser);

    const auto& result = marker.getCommands();
    EXPECT_EQ(result, expected);
}

TEST(RoadMarker, FillerInvalid) {
    std::istringstream ss{
        "20 contin\n"
        ".././/dfa\n"
        "empty 30\n" };
    Parser parser(ss);
    RoadMarker marker;
    EXPECT_ANY_THROW(marker.markupFiller(parser));
    EXPECT_ANY_THROW(marker.getCommands());
}

TEST(Writer, Right) {
    std::vector<Command> result = {
        Command{1.0, up}, Command{20.0, down}, Command{24.0, up}, Command{27.0, down}
    };
    std::ostringstream oss;
    Writer writer(oss);
    writer.writeRoadMarking(result);
    auto expect = "1 up\n20 down\n24 up\n27 down\n";
    EXPECT_EQ(oss.str(), expect);
}

TEST(Writer, Empty) {
    std::vector<Command> result;
    std::ostringstream oss;
    Writer writer(oss);
    writer.writeRoadMarking(result);
    EXPECT_EQ(oss.str(), "");
}
