from binary_search import binary_search

def run_tests():
    passed = 0
    total = 0

    def test_empty_array():
        nonlocal passed, total
        total += 1
        result = binary_search([], 5)
        assert result == -1
        print("Test empty array: PASSED")
        return 1

    def test_single_element():
        nonlocal passed, total
        total += 2
        result1 = binary_search([5], 5)
        assert result1 == 0
        print("Test single element (found): PASSED")
        result2 = binary_search([3], 5)
        assert result2 == -1
        print("Test single element (not found): PASSED")
        return 2

    def test_even_length():
        nonlocal passed, total
        total += 3
        arr = [1, 3, 5, 7]
        result1 = binary_search(arr, 3)
        assert result1 == 1
        print("Test even length (first half): PASSED")
        result2 = binary_search(arr, 5)
        assert result2 == 2
        print("Test even length (second half): PASSED")
        result3 = binary_search(arr, 4)
        assert result3 == -1
        print("Test even length (not found): PASSED")
        return 3

    def test_odd_length():
        nonlocal passed, total
        total += 3
        arr = [2, 4, 6, 8, 10]
        result1 = binary_search(arr, 4)
        assert result1 == 1
        print("Test odd length (first half): PASSED")
        result2 = binary_search(arr, 8)
        assert result2 == 3
        print("Test odd length (second half): PASSED")
        result3 = binary_search(arr, 7)
        assert result3 == -1
        print("Test odd length (not found): PASSED")
        return 3

    def test_duplicates():
        nonlocal passed, total
        total += 2
        arr = [1, 2, 2, 2, 3]
        result1 = binary_search(arr, 2)
        assert result1 in [1, 2, 3]  # Может вернуть любой из индексов
        print("Test duplicates (found): PASSED")
        result2 = binary_search(arr, 4)
        assert result2 == -1
        print("Test duplicates (not found): PASSED")
        return 2

    try:
        passed += test_empty_array()
        passed += test_single_element()
        passed += test_even_length()
        passed += test_odd_length()
        passed += test_duplicates()
        print(f"\nAll tests passed! {passed}/{total} tests successful.")
    except AssertionError as e:
        print(f"Test failed: {e}")
        print(f"Tests passed: {passed}/{total}")

if __name__ == "__main__":
    run_tests()
