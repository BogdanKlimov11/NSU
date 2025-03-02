#pragma once

#include <algorithm>
#include <vector>
#include <iterator>

namespace merge {
    namespace {
        template <class Iterator, class Compare>
        void myMerge(Iterator first, Iterator split, Iterator last, Compare comp) {
            while (first != split && split != last) {
                auto iter = split;
                first = std::find_if(first, split, [&split, &comp](const typename std::iterator_traits<Iterator>::value_type& it) { return comp(*split, it); });
                split = std::find_if(split, last, [&first, &comp](const typename std::iterator_traits<Iterator>::value_type& it) { return comp(*first, it); });
                std::rotate(first, iter, split);
            }
        }

        template <class Iterator, class Compare>
        void mergeSort(Iterator first, Iterator last, Compare comp) {
            auto size = std::distance(first, last);
            if (size < 2) {
                return;
            }
            auto split = first + size / 2;
            mergeSort(first, split, comp);
            mergeSort(split, last, comp);
            myMerge(first, split, last, comp);
        }
    }

    template <class Iterator, class Compare = std::less<>>
    std::enable_if_t<std::is_same_v<typename std::iterator_traits<Iterator>::iterator_category, std::random_access_iterator_tag>, void>
    sort(Iterator first, Iterator last, Compare comp = Compare()) {
        mergeSort(first, last, comp);
    }
}
