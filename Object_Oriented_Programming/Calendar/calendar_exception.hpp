#pragma once

#include <string>
#include <iostream>
#include <utility>

class CalendarException : std::exception {
protected:
    virtual void Msg() = 0;
};

class CalendarExceptionFile final : CalendarException {
private:
    std::string FileName;

public:
    explicit CalendarExceptionFile(const char FormatFileName[] = "Undefined file name") : FileName(FormatFileName) {}
    inline void Msg() override { std::cerr << "Your file " << FileName << " could not be opened" << std::endl; }
};

class CalendarExceptionFormat final : CalendarException {
private:
    std::string Format;

public:
    explicit CalendarExceptionFormat(std::string WrongFormat = "Undefined") : Format(std::move(WrongFormat)) {}
    inline void Msg() override { std::cerr << Format << std::endl; }
};
