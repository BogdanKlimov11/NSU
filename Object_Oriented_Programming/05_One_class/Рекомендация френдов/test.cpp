#include "gtest.h"
#include "friend_recommendation.h"
#include "parser.h"
#include "writer.h"

TEST(FriendRecommendationTest, Filler) {
    std::istringstream ss{
        "id1 id2\n"
        "id2 id3\n"
        "id2 id4\n"
        "id1 id3\n"
        "id1 id1\n"
        "id3 id4\n" };
    Parser parser(ss);
    FriendRecommendation fr;
    fr.friendFiller(parser);

    const auto& result1 = fr.getFriendsOf("id1");
    const auto& result2 = fr.getFriendsOf("id2");
    const auto& result4 = fr.getFriendsOf("id4");
    std::unordered_set<std::string> expected1{ "id2", "id3" };
    std::unordered_set<std::string> expected2{ "id1", "id3", "id4" };
    std::unordered_set<std::string> expected4{ "id2", "id3" };

    EXPECT_EQ(result1, expected1);
    EXPECT_EQ(result2, expected2);
    EXPECT_EQ(result4, expected4);
}

TEST(FriendRecommendationTest, FillerInvalid) {
    std::istringstream ss{
        "id2\n"
        ".././/dfa\n"
        "id1 id2 sdf\n" };
    Parser parser(ss);
    FriendRecommendation fr;
    fr.friendFiller(parser);

    const auto& result1 = fr.getFriendsOf("id1");
    const auto& result2 = fr.getFriendsOf("id2");
    std::unordered_set<std::string> expected1{ "id2" };
    std::unordered_set<std::string> expected2{ "id1" };
    EXPECT_EQ(result1, expected1);
    EXPECT_EQ(result2, expected2);
}

TEST(FriendRecommendationTest, GetFriendsOf) {
    FriendRecommendation fr;
    fr.AddFriendship("id1", "id2");
    fr.AddFriendship("id1", "id3");
    fr.AddFriendship("id1", "id4");
    fr.AddFriendship("id2", "id3");
    fr.AddFriendship("id2", "id7");
    fr.AddFriendship("id2", "id2");

    const auto& result1 = fr.getFriendsOf("id1");
    const auto& result2 = fr.getFriendsOf("id2");
    std::unordered_set<std::string> expected1{ "id2", "id3", "id4" };
    std::unordered_set<std::string> expected2{ "id1", "id3", "id7" };

    EXPECT_EQ(result1, expected1);
    EXPECT_EQ(result2, expected2);
}

TEST(FriendRecommendationTest, AddAndRecommend) {
    FriendRecommendation fr;
    fr.AddFriendship("id1", "id2");
    fr.AddFriendship("id1", "id3");
    fr.AddFriendship("id1", "id4");
    fr.AddFriendship("id2", "id3");
    fr.AddFriendship("id2", "id7");
    fr.AddFriendship("id2", "id2");

    auto result1 = fr.RecommendTo("id1", 0);
    auto result7 = fr.RecommendTo("id7", 100);
    std::vector<std::string> expected1{"id1", "id7"};
    std::vector<std::string> expected7{"id7", "id1", "id3" };

    EXPECT_EQ(result1, expected1);
    EXPECT_EQ(result7, expected7);
    EXPECT_ANY_THROW(fr.getFriendsOf("invalid"));
}

TEST(FriendRecommendationTest, MethodsInvalid) {
    FriendRecommendation fr;
    EXPECT_NO_THROW(fr.AddFriendship("id1", "id1"));
    EXPECT_NO_THROW(fr.AddFriendship("id3", "id3"));
    EXPECT_ANY_THROW(fr.AddFriendship("", ""));
    fr.AddFriendship("id7", "id8");
    EXPECT_ANY_THROW(fr.RecommendTo("", 80));
    EXPECT_ANY_THROW(fr.RecommendTo("id1", 80));
    EXPECT_ANY_THROW(fr.RecommendTo("id7", 110));
}

TEST(Streams, ParserRight) {
    std::istringstream ss{
        "id1 id2\n"
        "id2 id3\n"
        "id2 id4\n"
        "id1 id3\n"
        "id1 id1\n"
        "id3 id4\n"};
    Parser parser(ss);

    auto expect1 = std::make_pair<std::string, std::string>("id1", "id2");
    auto expect2 = std::make_pair<std::string, std::string>("id2", "id3");
    auto expect3 = std::make_pair<std::string, std::string>("id2", "id4");
    auto expect4 = std::make_pair<std::string, std::string>("id1", "id3");
    auto expect5 = std::make_pair<std::string, std::string>("id1", "id1");
    auto expect6 = std::make_pair<std::string, std::string>("id3", "id4");

    auto friends1 = parser.getNextFriendship();
    auto friends2 = parser.getNextFriendship();
    auto friends3 = parser.getNextFriendship();
    auto friends4 = parser.getNextFriendship();
    auto friends5 = parser.getNextFriendship();
    auto friends6 = parser.getNextFriendship();

    EXPECT_EQ(friends1.first, expect1);
    EXPECT_EQ(friends2.first, expect2);
    EXPECT_EQ(friends3.first, expect3);
    EXPECT_EQ(friends4.first, expect4);
    EXPECT_EQ(friends5.first, expect5);
    EXPECT_EQ(friends6.first, expect6);

    EXPECT_EQ(friends1.second, true);
    EXPECT_EQ(friends2.second, true);
    EXPECT_EQ(friends3.second, true);
    EXPECT_EQ(friends4.second, true);
    EXPECT_EQ(friends5.second, true);
    EXPECT_EQ(friends6.second, true);
}

TEST(Streams, ParserInvalid) {
    std::istringstream ss{
        "id2\n"
        ".././/dfa\n"
        "id1 id2 sdf\n" };
    Parser parser(ss);
    auto expect1 = std::make_pair<std::string, std::string>("id1", "id2");
    auto empty = std::make_pair<std::string, std::string>("", "");

    auto friends = parser.getNextFriendship();
    auto emptyfriends = parser.getNextFriendship();
    EXPECT_EQ(friends.first, expect1);
    EXPECT_EQ(friends.second, true);
    EXPECT_EQ(emptyfriends.first, empty);
    EXPECT_EQ(emptyfriends.second, false);
}

TEST(Streams, WriterRight) {
    std::vector<std::string> result = {
        "id0",
        "id1",
        "id2",
        "id3",
        "id4"
    };

    std::ostringstream oss;
    Writer writer(oss);
    writer.writeRecommendedFriends(result);
    auto expect =
        "Recommendation for id0\n"
        "id1\n"
        "id2\n"
        "id3\n"
        "id4\n";

    EXPECT_EQ(oss.str(), expect);
}

TEST(Streams, WriterEmpty) {
    std::vector<std::string> result;
    std::ostringstream oss;
    Writer writer(oss);
    writer.writeRecommendedFriends(result);
    EXPECT_EQ(oss.str(), "");
}

TEST(RecommendTo, percent100) {
    std::istringstream ss{
        "id1 id2\n"
        "id1 id5\n"
        "id5 id10\n"
        "id2 id3\n"
        "id2 id4\n"
        "id1 id3\n"
        "id1 id1\n"
        "id3 id4\n"
        "id3 id7\n"
        "id3 id8\n"
        "id3 id9\n" 
        "id5 id4\n" };
    Parser parser(ss);
    FriendRecommendation fr;
    fr.friendFiller(parser);

    auto result = fr.RecommendTo("id1", 100);
    std::vector<std::string> expected{ "id1", "id4"};

    EXPECT_EQ(result, expected);
}

TEST(RecommendTo, percent0) {
    std::istringstream ss{
        "id1 id2\n"
        "id1 id5\n"
        "id5 id10\n"
        "id2 id3\n"
        "id2 id4\n"
        "id1 id3\n"
        "id1 id1\n"
        "id3 id4\n"
        "id3 id7\n"
        "id3 id8\n"
        "id3 id9\n"
        "id5 id4\n"
        "id2 id7\n" };
    Parser parser(ss);
    FriendRecommendation fr;
    fr.friendFiller(parser);

    auto result = fr.RecommendTo("id9", 0);
    std::vector<std::string> expected{"id9", "id2" ,"id1", "id4", "id7", "id8"};

    EXPECT_EQ(result, expected);
}

TEST(RecommendTo, percent30) {
    std::istringstream ss{
        "id1 id2\n"
        "id1 id5\n"
        "id5 id10\n"
        "id2 id3\n"
        "id2 id4\n"
        "id1 id3\n"
        "id1 id1\n"
        "id3 id4\n"
        "id3 id7\n"
        "id3 id8\n"
        "id3 id9\n"
        "id5 id4\n"
        "id2 id7\n" };
    Parser parser(ss);
    FriendRecommendation fr;
    fr.friendFiller(parser);
    /*
    id1: id2 id3 id5
    id2: id1 id3 id4 id7
    id3: id1 id2 id4 id7 id8 id9
    id5: id10 id4
    */
    auto result = fr.RecommendTo("id1", 30);
    std::vector<std::string> expected{ "id1", "id4", "id7", "id10", "id9", "id8" };

    EXPECT_EQ(result, expected);
}

TEST(RecommendTo, percent50) {
    std::istringstream ss{
        "id1 id2\n"
        "id1 id5\n"
        "id5 id10\n"
        "id2 id3\n"
        "id2 id4\n"
        "id1 id3\n"
        "id1 id1\n"
        "id3 id4\n"
        "id3 id7\n"
        "id3 id8\n"
        "id3 id9\n"
        "id5 id4\n"
        "id2 id7\n" };
    Parser parser(ss);
    FriendRecommendation fr;
    fr.friendFiller(parser);
    /*
    id1: id2 id3 id5
    id2: id1 id3 id4 id7
    id3: id1 id2 id4 id7 id8 id9
    id5: id10 id4
    */
    auto result = fr.RecommendTo("id1", 50);
    std::vector<std::string> expected{ "id1", "id4", "id7" };

    EXPECT_EQ(result, expected);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
