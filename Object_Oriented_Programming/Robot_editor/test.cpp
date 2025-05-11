#include <gtest/gtest.h>

#include "editor.hpp"
#include "parser.hpp"
#include "utilities.hpp"

TEST(TextEditor, DELETE_LINES) {
    std::vector<std::string> text1{"I", "said", "What???", "Rock-n-roll", "Steeeeeeeeeve", "J"};
    std::vector<std::string> result1(text1.size());
    Editor editor(text1);
    editor.deleteLines(1, 0);
    std::stringstream out;
    out << editor;
    int i = 0;
    while (i < text1.size()) {
        out >> result1[i];
        i++;
    }
    ASSERT_TRUE(result1 == text1);
    editor.deleteLines(1, 2);
    std::vector<std::string> text2{"What???", "Rock-n-roll", "Steeeeeeeeeve", "J"};
    std::vector<std::string> result2(text2.size());
    std::stringstream out2;
    out2 << editor;
    i = 0;
    while (i < text2.size()) {
        out2 >> result2[i];
        i++;
    }
    ASSERT_TRUE(result2 == text2);
}

TEST(TextEditor, INSERT_LINES) {
    std::vector<std::string> text1{"Skyshay bulok", "Da vipey chai"};
    std::string text2 = "(Hello, World!)\nIt is me";
    std::string resText{"Skyshay bulok\n(Hello, World!)\nIt is me\nDa vipey chai\n"};
    Editor editor(text1);
    editor.insertLines(1, text2);
    editor.insertLines(300, text2);
    std::stringstream out;
    out << editor;
    ASSERT_TRUE(out.str() == resText);
}

TEST(TextEditor, CHANGE_LINES) {
    std::vector<std::string> text1{"A", "B", "C", "D", "E"};
    std::string text2 = "F\nG";
    Editor editor(text1);
    editor.changeLines(1, 3, text2);
    std::string resText1 = {"F\nG\nD\nE\n"};
    std::stringstream out1;
    out1 << editor;
    ASSERT_TRUE(out1.str() == resText1);
}

TEST(TextEditor, REPLACE_LINES) {
    std::vector<std::string> text1{"The squalling cat \"and the squeaking mouse,", "The \"howling \"dog by the door of the house"};
    std::string resText = {"The squalling cat and the squeaking mouse,\nThe howling dog by the door of the house\n"};
    std::string oldSubstr = "\"";
    std::string newSubstr = "";
    Editor editor(text1);
    editor.replaceLines(1, 3, oldSubstr, newSubstr);
    std::stringstream out;
    out << editor;
    ASSERT_TRUE(out.str() == resText);
}

TEST(TextEditor, DELETE) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"delete from 2 to 3"};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    auto expect = "A\nD\nE\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, DELETE_ALL) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"delete"};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    auto expect = "";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, REPLACE_ONCE) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"replace \"C\\nD\" with \"D\\nC\""};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "A\nB\nD\nC\nE\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, REPLACE_TWICE) {
    std::istringstream in{"A\nB\nC\"D\nE\nB\nC\"D\nB\nC\"D"};
    std::istringstream inCommands{"replace \"C\\\"D\" with \"D\\\"C\""};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "A\nB\nD\"C\nE\nB\nD\"C\nB\nD\"C\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, INSERT_ONCE) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"insert after 5 \"F\\nG\""};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "A\nB\nC\nD\nE\nF\nG\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, INSERT_TWICE) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"insert after 1 \"\\\\\""};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "A\n\\\nB\nC\nD\nE\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, CHANGE_ONCE) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"change from 1 to 4 with \"X\\nY\\nZ\\n\""};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "X\nY\nZ\nE\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, CHANGE_TWICE) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"change with \"X\\nY\\nZ\\n\""};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "X\nY\nZ\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(TextEditor, UNDO) {
    std::istringstream in{"A\nB\nC\nD\nE"};
    std::istringstream inCommands{"change with \"X\\nY\\nZ\\n\"\nundo"};
    auto input = loadFromStream(in);
    Editor editor = Editor(input);
    auto recipe = Command::readRecipe(inCommands);
    editor.applyRecipe(recipe);
    std::ostringstream out;
    out << editor;
    std::string expect = "A\nB\nC\nD\nE\n";
    EXPECT_EQ(out.str(), expect);
}

TEST(CommandParse, DELETE) {
    Command command;
    std::string line1 = "delete";
    ASSERT_TRUE(parseDelete(createWordList(line1), command));
    std::string line2 = "delete from 1 to 10";
    ASSERT_TRUE(parseDelete(createWordList(line2), command));
    std::string line3 = "delete from 1 to 10";
    ASSERT_TRUE(parseDelete(createWordList(line3), command));
    std::string line4 = "delete from to 10";
    ASSERT_FALSE(parseDelete(createWordList(line4), command));
}

TEST(CommandParse, WRONG_UNDO) {
    Command command;
    std::string line1 = "undo";
    ASSERT_TRUE(parseUndo(createWordList(line1), command));
    std::string line2 = "undo to";
    ASSERT_FALSE(parseUndo(createWordList(line2), command));
}

TEST(CommandParse, INSERT_CHAIN) {
    Command command;
    std::string line1 = "insert after 1 \"Hi!\\nI'm FG\"";
    ASSERT_TRUE(parseInsert(createWordList(line1), command));
    std::string line2 = "insert 1 \"Hi!\"";
    ASSERT_FALSE(parseInsert(createWordList(line2), command));
    std::string line3 = "insert after 1 \"Hi!";
    ASSERT_FALSE(parseInsert(createWordList(line3), command));
    std::string line4 = "insert after 1 Hi!\"";
    ASSERT_FALSE(parseInsert(createWordList(line4), command));
}

TEST(CommandParse, CHANGE_CHAIN) {
    Command command;
    std::string line1 = "change from 1 to 10 with \"\"";
    ASSERT_TRUE(parseChange(createWordList(line1), command));
    std::string line2 = "change to 10 with \"\"";
    ASSERT_TRUE(parseChange(createWordList(line2), command));
    std::string line3 = "change from 1 to with \"Hi!";
    ASSERT_FALSE(parseChange(createWordList(line3), command));
    std::string line4 = "change from 1 to 10 with \"";
    ASSERT_FALSE(parseChange(createWordList(line4), command));
}

TEST(CommandParse, REPLACE_CHAIN) {
    Command command;
    std::string line1 = "replace from 1 to 10 \"A\" with \"B\"";
    ASSERT_TRUE(parseReplace(createWordList(line1), command));
    std::string line2 = "replace to 10 \"A\" with \"B\"";
    ASSERT_TRUE(parseReplace(createWordList(line2), command));
    std::string line3 = "replace \"A\" with \"B\"";
    ASSERT_TRUE(parseReplace(createWordList(line3), command));
    std::string line4 = "replace from 1 to 10 \"A\" \"B\"";
    ASSERT_FALSE(parseReplace(createWordList(line4), command));
    std::string line5 = "replace from 1 to 10 \"A\" with B\"";
    ASSERT_FALSE(parseReplace(createWordList(line5), command));
}

TEST(words, SEQUENCE_OF_COMMANDS) {
    std::istringstream inCommands{"replace \" C\\nD       \" with \" D\\nC \"\n"
                                  "insert after 1 \"Hi!\\nI'm FG         \"\n"
                                  "change with  \" X\\nY\\nZ\\n \""};
    auto recipe = Command::readRecipe(inCommands);
    ASSERT_EQ(recipe.size(), 3);
}
