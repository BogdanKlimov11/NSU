#pragma once

#include <map>
#include <list>
#include <forward_list>
#include <set>

template<typename T>
class Traits;

template<>
class Traits<unsigned char> {
public:
    typedef int Trait;
};

template<>
class Traits<char> {
public:
    typedef std::string Trait;
};

template<>
class Traits<int> {
public:
    typedef long Trait;
};

template<>
class Traits<double> {
public:
    typedef long double Trait;
};

template<>
class Traits<std::string> {
public:
    typedef std::string Trait;
};

template<>
class Traits<short> {
public:
    typedef int Trait;
};

template<>
class Traits<long> {
public:
    typedef long long Trait;
};

template<>
class Traits<float> {
public:
    typedef double Trait;
};

template<>
class Traits<long double> {
public:
    typedef long double Trait;
};

template<typename T, typename G>
class Traits<std::map<T, G>> {
public:
    typedef typename G Trait;
};

template<typename T>
class Traits<std::list<T>> {
public:
    typedef typename T Trait;
};

template<>
class Traits<unsigned int> {
public:
    typedef unsigned long Trait;
};

template<>
class Traits<unsigned short> {
public:
    typedef unsigned int Trait;
};

template<>
class Traits<unsigned long> {
public:
    typedef unsigned long long Trait;
};
