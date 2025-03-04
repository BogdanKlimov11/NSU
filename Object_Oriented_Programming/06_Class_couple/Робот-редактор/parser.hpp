#ifndef LAB_REDACTOR_PARSER_HPP
#define LAB_REDACTOR_PARSER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ios>

#include "command.hpp"

std::vector<std::string> loadFromStream(std::istream& input);
bool parseDelete(const std::vector<std::string>& words, Command& command);
bool parseUndo(const std::vector<std::string>& words, Command& command);
bool parseInsert(const std::vector<std::string>& words, Command& command);
bool parseChange(const std::vector<std::string>& words, Command& command);
bool parseReplace(const std::vector<std::string>& words, Command& command);

#endif
