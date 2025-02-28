#pragma once

#include "filter_iterator.hpp"

template<typename Predicate, typename Iter>
class Range {
private:
    std::optional<Predicate> m_pred;
    filter_iterator<Predicate, Iter> m_begin;
    filter_iterator<Predicate, Iter> m_end;

public:
    Range(Predicate pred_, Iter begin_, Iter end_) : m_pred(pred_),
        m_begin(filter_iterator<Predicate, Iter>(pred_, begin_, end_)),
        m_end(filter_iterator<Predicate, Iter>()) {}

    Range(Iter begin_, Iter end_, typename std::enable_if<
        std::is_default_constructible_v<Predicate> && std::is_class_v<Predicate>>::type* = 0) :
        m_pred(Predicate()), m_begin(filter_iterator<Predicate, Iter>(begin_, end_)),
        m_end(filter_iterator<Predicate, Iter>()) {}

    filter_iterator<Predicate, Iter> begin() const {
        return m_begin;
    }

    filter_iterator<Predicate, Iter> end() const {
        return m_end;
    }
};
