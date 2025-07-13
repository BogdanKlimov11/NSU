from z_function import substring_search

def run_tests():
    passed = 0
    total = 0

    def test_empty_pattern():
        nonlocal passed, total
        total += 1
        result = substring_search("abc", "")
        assert result == [0, 1, 2, 3]
        print("Test empty pattern: PASSED")
        return 1

    def test_empty_text():
        nonlocal passed, total
        total += 1
        result = substring_search("", "a")
        assert result == []
        print("Test empty text: PASSED")
        return 1

    def test_pattern_longer():
        nonlocal passed, total
        total += 1
        result = substring_search("ab", "abc")
        assert result == []
        print("Test pattern longer: PASSED")
        return 1

    def test_single_match():
        nonlocal passed, total
        total += 1
        result = substring_search("abcde", "bcd")
        assert result == [1]
        print("Test single match: PASSED")
        return 1

    def test_multiple_matches():
        nonlocal passed, total
        total += 1
        result = substring_search("ababab", "aba")
        assert result == [0, 2]
        print("Test multiple matches: PASSED")
        return 1

    def test_no_match():
        nonlocal passed, total
        total += 1
        result = substring_search("abcdef", "xyz")
        assert result == []
        print("Test no match: PASSED")
        return 1

    def test_pattern_equals_text():
        nonlocal passed, total
        total += 1
        result = substring_search("abc", "abc")
        assert result == [0]
        print("Test pattern equals text: PASSED")
        return 1

    def test_overlapping():
        nonlocal passed, total
        total += 1
        result = substring_search("aaa", "aa")
        assert result == [0, 1]
        print("Test overlapping: PASSED")
        return 1

    def test_unicode():
        nonlocal passed, total
        total += 1
        result = substring_search("привет мир", "мир")
        assert result == [7]
        print("Test unicode: PASSED")
        return 1

    try:
        passed += test_empty_pattern()
        passed += test_empty_text()
        passed += test_pattern_longer()
        passed += test_single_match()
        passed += test_multiple_matches()
        passed += test_no_match()
        passed += test_pattern_equals_text()
        passed += test_overlapping()
        passed += test_unicode()
        print(f"\nAll tests passed! {passed}/{total} tests successful.")
    except AssertionError as e:
        print(f"Test failed: {e}")
        print(f"Tests passed: {passed}/{total}")

if __name__ == "__main__":
    run_tests()
