import random
from heap_sort import heap_sort

def is_sorted(arr):
    return all(arr[i] <= arr[i + 1] for i in range(len(arr) - 1))

def run_tests():
    passed = 0
    total = 0

    def test_empty_array():
        nonlocal passed, total
        total += 1
        result = heap_sort([])
        assert result == [], "Empty array should return empty"
        print("Test empty array: PASSED")
        return 1

    def test_single_element():
        nonlocal passed, total
        total += 1
        result = heap_sort([1])
        assert result == [1], "Single element array should return same element"
        print("Test single element: PASSED")
        return 1

    def test_two_elements():
        nonlocal passed, total
        total += 2
        result1 = heap_sort([2, 1])
        assert result1 == [1, 2], "Two elements should be sorted"
        print("Test two elements (unsorted): PASSED")
        result2 = heap_sort([1, 2])
        assert result2 == [1, 2], "Already sorted two elements should stay sorted"
        print("Test two elements (sorted): PASSED")
        return 2

    def test_even_length():
        nonlocal passed, total
        total += 2
        result1 = heap_sort([4, 2, 3, 1])
        assert result1 == [1, 2, 3, 4], "Even length array should be sorted"
        print("Test even length (unsorted): PASSED")
        result2 = heap_sort([1, 2, 3, 4])
        assert result2 == [1, 2, 3, 4], "Already sorted even length should stay sorted"
        print("Test even length (sorted): PASSED")
        return 2

    def test_odd_length():
        nonlocal passed, total
        total += 2
        result1 = heap_sort([3, 1, 4, 1, 5])
        assert result1 == [1, 1, 3, 4, 5], "Odd length array should be sorted"
        print("Test odd length (unsorted): PASSED")
        result2 = heap_sort([1, 2, 3, 4, 5])
        assert result2 == [1, 2, 3, 4, 5], "Already sorted odd length should stay sorted"
        print("Test odd length (sorted): PASSED")
        return 2

    def test_same_elements():
        nonlocal passed, total
        total += 2
        result1 = heap_sort([2, 2, 2, 2])
        assert result1 == [2, 2, 2, 2], "Array with same elements should remain unchanged"
        print("Test same elements (four): PASSED")
        result2 = heap_sort([1, 1, 1])
        assert result2 == [1, 1, 1], "Array with same elements should remain unchanged"
        print("Test same elements (three): PASSED")
        return 2

    def test_reverse_sorted():
        nonlocal passed, total
        total += 1
        result = heap_sort([5, 4, 3, 2, 1])
        assert result == [1, 2, 3, 4, 5], "Reverse sorted array should be sorted"
        print("Test reverse sorted: PASSED")
        return 1

    def test_random_arrays():
        nonlocal passed, total
        random_tests = 100
        total += random_tests
        for i in range(random_tests):
            size = random.randint(0, 100)
            arr = [random.randint(-1000, 1000) for _ in range(size)]
            sorted_arr = heap_sort(arr)
            assert is_sorted(sorted_arr), f"Random array of size {size} should be sorted"
            assert len(sorted_arr) == len(arr), f"Random array length should be preserved"
            assert sorted(set(arr)) == sorted(set(sorted_arr)), f"Random array elements should be preserved"
            print(f"Test random array {i + 1}/{random_tests}: PASSED")
        return random_tests

    try:
        passed += test_empty_array()
        passed += test_single_element()
        passed += test_two_elements()
        passed += test_even_length()
        passed += test_odd_length()
        passed += test_same_elements()
        passed += test_reverse_sorted()
        passed += test_random_arrays()
        print(f"\nAll tests passed! {passed}/{total} tests successful.")
    except AssertionError as e:
        print(f"Test failed: {e}")
        print(f"Tests passed: {passed}/{total}")

if __name__ == "__main__":
    run_tests()
