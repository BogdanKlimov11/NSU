import random
import pytest
from heap_sort import heap_sort

def is_sorted(arr):
    return all(arr[i] <= arr[i + 1] for i in range(len(arr) - 1))

def test_empty_array():
    assert heap_sort([]) == [], "Empty array should return empty"

def test_single_element():
    assert heap_sort([1]) == [1], "Single element array should return same element"

def test_two_elements():
    assert heap_sort([2, 1]) == [1, 2], "Two elements should be sorted"
    assert heap_sort([1, 2]) == [1, 2], "Already sorted two elements should stay sorted"

def test_even_length():
    assert heap_sort([4, 2, 3, 1]) == [1, 2, 3, 4], "Even length array should be sorted"
    assert heap_sort([1, 2, 3, 4]) == [1, 2, 3, 4], "Already sorted even length should stay sorted"

def test_odd_length():
    assert heap_sort([3, 1, 4, 1, 5]) == [1, 1, 3, 4, 5], "Odd length array should be sorted"
    assert heap_sort([1, 2, 3, 4, 5]) == [1, 2, 3, 4, 5], "Already sorted odd length should stay sorted"

def test_same_elements():
    assert heap_sort([2, 2, 2, 2]) == [2, 2, 2, 2], "Array with same elements should remain unchanged"
    assert heap_sort([1, 1, 1]) == [1, 1, 1], "Array with same elements should remain unchanged"

def test_reverse_sorted():
    assert heap_sort([5, 4, 3, 2, 1]) == [1, 2, 3, 4, 5], "Reverse sorted array should be sorted"

def test_random_arrays():
    for _ in range(100):
        size = random.randint(0, 100)
        arr = [random.randint(-1000, 1000) for _ in range(size)]
        sorted_arr = heap_sort(arr)
        assert is_sorted(sorted_arr), f"Random array of size {size} should be sorted"
        assert len(sorted_arr) == len(arr), f"Random array length should be preserved"
        assert sorted(set(arr)) == sorted(set(sorted_arr)), f"Random array elements should be preserved"

if __name__ == "__main__":
    pytest.main()
