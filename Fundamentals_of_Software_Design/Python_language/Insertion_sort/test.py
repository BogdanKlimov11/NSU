import random
from insertion_sort import insertion_sort

def is_sorted(arr):
    return all(arr[i] <= arr[i + 1] for i in range(len(arr) - 1))

def run_tests():
    passed = 0
    total = 0

    def test_empty_array():
        nonlocal passed, total
        total += 1
        arr = []
        insertion_sort(arr)
        assert arr == []
        print("Test empty array: PASSED")
        return 1

    def test_single_element():
        nonlocal passed, total
        total += 1
        arr = [1]
        insertion_sort(arr)
        assert arr == [1]
        print("Test single element: PASSED")
        return 1

    def test_two_elements():
        nonlocal passed, total
        total += 2
        arr1 = [2, 1]
        insertion_sort(arr1)
        assert arr1 == [1, 2]
        print("Test two elements (unsorted): PASSED")
        arr2 = [1, 2]
        insertion_sort(arr2)
        assert arr2 == [1, 2]
        print("Test two elements (sorted): PASSED")
        return 2

    def test_even_length():
        nonlocal passed, total
        total += 2
        arr1 = [4, 2, 3, 1]
        insertion_sort(arr1)
        assert arr1 == [1, 2, 3, 4]
        print("Test even length (unsorted): PASSED")
        arr2 = [1, 2, 3, 4]
        insertion_sort(arr2)
        assert arr2 == [1, 2, 3, 4]
        print("Test even length (sorted): PASSED")
        return 2

    def test_odd_length():
        nonlocal passed, total
        total += 2
        arr1 = [3, 1, 4, 1, 5]
        insertion_sort(arr1)
        assert arr1 == [1, 1, 3, 4, 5]
        print("Test odd length (unsorted): PASSED")
        arr2 = [1, 2, 3, 4, 5]
        insertion_sort(arr2)
        assert arr2 == [1, 2, 3, 4, 5]
        print("Test odd length (sorted): PASSED")
        return 2

    def test_same_elements():
        nonlocal passed, total
        total += 2
        arr1 = [2, 2, 2, 2]
        insertion_sort(arr1)
        assert arr1 == [2, 2, 2, 2]
        print("Test same elements (four): PASSED")
        arr2 = [1, 1, 1]
        insertion_sort(arr2)
        assert arr2 == [1, 1, 1]
        print("Test same elements (three): PASSED")
        return 2

    def test_reverse_sorted():
        nonlocal passed, total
        total += 1
        arr = [5, 4, 3, 2, 1]
        insertion_sort(arr)
        assert arr == [1, 2, 3, 4, 5]
        print("Test reverse sorted: PASSED")
        return 1

    def test_random_arrays():
        nonlocal passed, total
        random_tests = 100
        total += random_tests
        for i in range(random_tests):
            size = random.randint(0, 100)
            arr = [random.randint(-1000, 1000) for _ in range(size)]
            original_elements = arr.copy()
            insertion_sort(arr)
            assert is_sorted(arr)
            assert len(arr) == len(original_elements)
            assert sorted(arr) == sorted(original_elements)
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
