#pragma once

#include <algorithm>
#include <vector>
#include <iterator>
#include <functional>
#include <future>
#include <cmath>

#include "policies.hpp"

namespace quick_sort {

    size_t max_thread_number = 18;

    template<class Iterator, class Compare>
    void quickSort(Iterator first, Iterator last, Compare comp) {
        auto size = std::distance(first, last);
        if (size < 2) {
            return;
        }
        auto pivot = *(first + size / 2);
        auto left = first;
        auto right = last - 1;

        while (true) {
            while (comp(*left, pivot)) {
                ++left;
            }
            while (comp(pivot, *right)) {
                --right;
            }
            if (left >= right) {
                break;
            }
            std::iter_swap(left++, right--);
        }

        quickSort(first, left, comp);
        quickSort(left, last, comp);
    }

    template<class Iterator, class Compare = std::less<>>
    std::enable_if_t<std::is_same_v<typename std::iterator_traits<Iterator>::iterator_category,
                                    std::random_access_iterator_tag>, void>
    sort(Iterator first, Iterator last, Compare comp = Compare(), const AbstractPolicy& policy = AutoPolicy()) {
        auto size = std::distance(first, last);
        auto thread_number = policy.get_threads_number(size);
        auto partition = static_cast<size_t>(round(static_cast<double>(size) / static_cast<double>(thread_number)));

        if (thread_number != 1) {
            std::vector<Iterator> splits;
            for (size_t i = 0; i < thread_number; ++i) {
                auto split = first + i * partition;
                splits.push_back(split);
            }
            splits.push_back(last);

            std::vector<std::future<void>> all_threads;
            for (size_t i = 0; i < splits.size() - 1; ++i) {
                all_threads.push_back(std::async(std::launch::async, quickSort<Iterator, Compare>,
                                                 std::ref(splits[i]), std::ref(splits[i + 1]), std::ref(comp)));
            }

            for (auto& thread : all_threads) {
                thread.wait();
            }

            for (size_t i = 0; i < splits.size() - 2; ++i) {
                std::inplace_merge(splits[0], splits[i + 1], splits[i + 2], comp);
            }
        } else {
            quickSort(first, last, comp);
        }
    }
}
