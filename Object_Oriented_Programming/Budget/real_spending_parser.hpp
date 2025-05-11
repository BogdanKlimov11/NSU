#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "spending.hpp"
#include "date.hpp"

class RealSpendingParser final {
private:
	std::istream& is;

public:
	RealSpendingParser(std::istream& _is) : is{ _is } {};
	std::pair<Spending, bool> getNextItem();
	std::pair<std::string, std::string> getPath(const std::string & str);
};
