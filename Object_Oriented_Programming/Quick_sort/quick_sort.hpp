#pragma once

#include <algorithm>
#include <vector>
#include <iterator>

namespace quick_sort {

template <class Iterator, class Compare>
void quickSort(Iterator first, Iterator last, Compare comp) {
    auto size = std::distance(first, last);
    if (size < 2) {
        return;
    }

    auto pivot = *first;
    auto left = first;
    auto right = last - 1;

    while (left < right) {
        while (comp(*left, pivot)) {
            ++left;
        }
        while (comp(pivot, *right)) {
            --right;
        }
        std::iter_swap(left, right);
    }

    quickSort(first, left, comp);
    quickSort(left + 1, last, comp);
}

template <class Iterator, class Compare = std::less<>>
std::enable_if_t<std::is_same_v<typename std::iterator_traits<Iterator>::iterator_category,
    std::random_access_iterator_tag>, void>
sort(Iterator first, Iterator last, Compare comp = Compare()) {
    quickSort(first, last, comp);
}

}
