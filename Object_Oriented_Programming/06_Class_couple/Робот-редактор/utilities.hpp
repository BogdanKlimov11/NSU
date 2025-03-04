#ifndef LAB_REDACTOR_UTILITIES_HPP
#define LAB_REDACTOR_UTILITIES_HPP

#include <string>
#include <vector>

bool inQuotes(const std::string &str);
void deleteQuotes(std::string &str);
int convertToInt(const std::string &numStr);
std::vector <std::string> createWordList(const std::string &str);

#endif
