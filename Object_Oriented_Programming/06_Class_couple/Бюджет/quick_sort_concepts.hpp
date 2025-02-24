#pragma once

#include <algorithm>
#include <vector>
#include <iterator>

namespace quick_sort {
    template<class Iterator, class Compare>
    void quickSort(Iterator first, Iterator last, Compare comp) {
        auto size = std::distance(first, last);
        if (size < 2) return;

        auto pivot = *first;
        auto left = first;
        auto right = last - 1;

        while (left < right) {
            while (comp(*left, pivot)) left++;
            while (comp(pivot, *right)) right--;
            std::iter_swap(left, right);
        }
        
        quickSort(first, left, comp);
        quickSort(left + 1, last, comp);
    }

    template<typename Iterator>
    concept RandomAccessIterator = std::bidirectional_iterator<Iterator> &&
        requires(const Iterator ci, const Iterator cj, Iterator i, Iterator j, const int n) {    
            { ci + n }  -> std::same_as<Iterator>;
            { ci - n }  -> std::same_as<Iterator>;
            { n + ci }  -> std::same_as<Iterator>;
            { i += n }  -> std::same_as<Iterator&>;
            { i -= n }  -> std::same_as<Iterator&>;
            { ci[n] }   -> std::same_as<decltype(*ci)>;
            { ci < cj } -> std::convertible_to<bool>;
            { ci <= cj } -> std::convertible_to<bool>;
            { ci > cj } -> std::convertible_to<bool>;
            { ci >= cj } -> std::convertible_to<bool>;
        };

    template <RandomAccessIterator Iterator, class Compare = std::less<>>
    void sort(Iterator first, Iterator last, Compare comp = Compare()) {
        quickSort(first, last, comp);
    }
}
