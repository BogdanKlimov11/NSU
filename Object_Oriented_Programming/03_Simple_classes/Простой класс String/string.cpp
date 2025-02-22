#include <algorithm>
#include <cstring>
#include <cassert>

#include "string.hpp"

namespace {
    size_t minLen(size_t l1, size_t l2) {
        return l2 < l1 ? l2 : l1;
    }
    const size_t MinCapacity = 10;
}

//class StringImpl
class StringImpl {
private:
    char* data_ = nullptr;
    size_t lenght_{};        // length of string
    size_t capacity_{};      // size of memory
    size_t countRef_{};      // reference counter

    void initialization(const char * str = nullptr, size_t len = 0, size_t capacity = MinCapacity);

public:
    StringImpl();
    StringImpl(const char* str, size_t n);
    StringImpl(size_t n, char c);
    StringImpl(const StringImpl& str, size_t pos, size_t len = npos);
    virtual ~StringImpl();

    void reduceRef();
    void addRef();
    size_t size() const;
    size_t capacity() const;
    void reserve(size_t n = 0);
    void clear();
    bool empty() const;

    char& at(size_t pos);
    const char& at(size_t pos) const;

    bool reallocateForReplace(size_t pos, size_t len, size_t strLen);
    void replace(size_t pos, size_t len, const char* str, size_t strLen);
    void replace(size_t pos, size_t len, size_t n, char c);

    const char* data() const;

    size_t find(const char* str, size_t pos = 0) const;
    StringImpl substr(size_t pos = 0, size_t len = npos) const;
    int compare(const StringImpl& str) const;
    int compare(const char* str) const;

    static const size_t npos = -1;

    size_t countRef() const;
    StringImpl* cloneIfNeed();
};

StringImpl* StringImpl::cloneIfNeed() {
    if (countRef() == 1) {
        return this;
    }
    StringImpl* tmp = new StringImpl(*this, 0);
    countRef_--;
    return tmp;
}

void StringImpl::initialization(const char * str, size_t len, size_t capacity) {
    if (capacity <= len) {
        capacity = len + 1;
    }

    char* data = data_;
    if (capacity_ != capacity) {
        capacity_ = capacity;
        data = new char[capacity_];
    }

    if (str && len != 0) {
        memmove(data, str, len);
    }

    if (data != data_) {
        delete[] data_;
    }

    data_ = data;
    data_[len] = '\0';
    lenght_ = len;
}

StringImpl::StringImpl() : data_(new char[MinCapacity]), lenght_(0), capacity_(MinCapacity), countRef_(1) {
    data_[lenght_] = '\0';
}

StringImpl::StringImpl(const char * str, size_t n) : StringImpl() {
    if (str) {
        size_t num = minLen(n, strlen(str));
        initialization(str, num, num + 1);
    }
}

StringImpl::StringImpl(size_t n, char c) : StringImpl() {
    initialization(nullptr, n, n + 1);
    if (data()) {
        memset(data_, c, n);
    }
}

StringImpl::StringImpl(const StringImpl& str, size_t pos, size_t len) {
    if (pos > str.lenght_) {
        throw std::out_of_range("Invalid argument: pos > lenght of string");
    }

    lenght_ = minLen(str.lenght_ - pos, len);
    countRef_ = 1;
    capacity_ = lenght_ + 1;
    data_ = new char[capacity_];
    memcpy(data_, str.data_ + pos, lenght_);
    data_[lenght_] = '\0';
}

StringImpl::~StringImpl() {
    delete[] data_;
}

void StringImpl::reduceRef() {
    if (--countRef_ <= 0)
        delete this;
}

void StringImpl::addRef() {
    countRef_++;
}

size_t StringImpl::size() const {
    return lenght_;
}

size_t StringImpl::capacity() const {
    return capacity_;
}

void StringImpl::reserve(size_t n) {
    assert(countRef_ == 1);
    initialization(data_, lenght_, n);
}

void StringImpl::clear() {
    assert(countRef_ == 1);
    data_[0] = '\0';
    lenght_ = 0;
}

bool StringImpl::empty() const {
    return (lenght_ == 0);
}

char & StringImpl::at(size_t pos) {
    assert(countRef_ == 1);
    if ((pos >= lenght_)) {
        throw std::out_of_range("Out of range: pos > lenght");
    }
    return data_[pos];
}

const char & StringImpl::at(size_t pos) const {
    if (pos > lenght_) {
        throw std::out_of_range("Out of range: pos > lenght");
    }

    return data_[pos];
}

bool StringImpl::reallocateForReplace(size_t pos, size_t len, size_t strLen) {
    assert(countRef_ == 1);
    if (pos > lenght_) {
        throw std::out_of_range("Invalid argument: can't replace");
    }

    const size_t min_len = minLen(len, lenght_ - pos);

    if (min_len == 0 && strLen == 0) {
        return false;
    }

    size_t newCapacity = lenght_ - min_len + strLen + 1;

    initialization(data(), lenght_, newCapacity);

    size_t restPos = pos + min_len;
    size_t restLen = size() - restPos;
    size_t newRestPos = pos + strLen;

    memmove(data_ + newRestPos, data_ + restPos, restLen);
    lenght_ = lenght_ - min_len + strLen;
    data_[lenght_] = 0;
    return true;
}

void StringImpl::replace(size_t pos, size_t len, const char* str, size_t strLen){
    if (reallocateForReplace(pos, len, strLen) && str) {
        memmove(data_ + pos, str, strLen);
    }
}

void StringImpl::replace(size_t pos, size_t len, size_t n, char c) {
    if (reallocateForReplace(pos, len, n)) {
        memset(data_ + pos, c, n);
    }
}

const char * StringImpl::data() const {
    return data_;
}

size_t StringImpl::find(const char * str, size_t pos) const {
    if ((pos >= lenght_) || (!str)) {
        return npos;
    }

    for (size_t i = pos; i <= lenght_ - strlen(str); i++) {
        if (memcmp(data_ + i, str, strlen(str)) == 0) {
            return i;
        }
    }
    return npos;
}

StringImpl StringImpl::substr(size_t pos, size_t len) const {
    if (pos >= lenght_) {
        throw std::out_of_range("Invalid agument: pos > lenght");
    }
    size_t n = minLen(len, lenght_ - pos);
    return StringImpl(data_ + pos, n);
}

int StringImpl::compare(const StringImpl & str) const {
    return compare(str.data_);
}

int StringImpl::compare(const char * str) const {
    if (!str) {
        return -1; 
    }

    for (size_t i = 0; i < lenght_; i++) {
        if (data_[i] > str[i]) {
            return 1;
        }

        if (data_[i] < str[i]) {
            return -1;
        }
    }
    if (lenght_ < strlen(str)) {
        return -1;
    }
    else if (lenght_ > strlen(str)) {
        return 1;
    }

    return 0;
}

size_t StringImpl::countRef() const {
    return countRef_;
}

//class String
String::String() : impl_(new StringImpl()){}

String::String(const char * str) {
    if(str) {
        impl_ = new StringImpl(str, npos);
    }
    else {
        impl_ = new StringImpl();
    }
}

String::String(const char * str, size_t n) {
    if (str) {
        impl_ = new StringImpl(str, n);
    }
    else {
        impl_ = new StringImpl();
    }
}

String::String(size_t n, char c) {
    impl_ = new StringImpl(n, c);
}

String::String(const String & str) {
    impl_ = str.impl_;
    str.impl_->addRef();
}

String::String(const String & str, size_t pos, size_t len) {
    if (pos == 0 && len == npos) {
        impl_ = str.impl_;
        str.impl_->addRef();
    }
    else {
        impl_ = new StringImpl(*str.impl_, pos, len);
    }
}

String::~String() {
    impl_->reduceRef();
}

size_t String::size() const {
    return impl_->size();
}

size_t String::
