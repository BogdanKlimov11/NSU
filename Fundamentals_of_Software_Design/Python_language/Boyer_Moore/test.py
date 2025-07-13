from boyer_moore import my_strstr

def run_tests():
    passed = 0
    total = 0

    def test_empty_needle():
        nonlocal passed, total
        total += 1
        result = my_strstr("any string", "")
        assert result == 0
        print("Test empty needle: PASSED")
        return 1

    def test_empty_haystack():
        nonlocal passed, total
        total += 1
        result = my_strstr("", "needle")
        assert result == -1
        print("Test empty haystack: PASSED")
        return 1

    def test_needle_longer_than_haystack():
        nonlocal passed, total
        total += 1
        result = my_strstr("short", "very long needle")
        assert result == -1
        print("Test needle longer than haystack: PASSED")
        return 1

    def test_exact_match():
        nonlocal passed, total
        total += 1
        result = my_strstr("hello world", "hello world")
        assert result == 0
        print("Test exact match: PASSED")
        return 1

    def test_substring_at_start():
        nonlocal passed, total
        total += 1
        result = my_strstr("hello world", "hello")
        assert result == 0
        print("Test substring at start: PASSED")
        return 1

    def test_substring_at_end():
        nonlocal passed, total
        total += 1
        result = my_strstr("hello world", "world")
        assert result == 6
        print("Test substring at end: PASSED")
        return 1

    def test_substring_in_middle():
        nonlocal passed, total
        total += 1
        result = my_strstr("hello beautiful world", "beautiful")
        assert result == 6
        print("Test substring in middle: PASSED")
        return 1

    def test_multiple_occurrences():
        nonlocal passed, total
        total += 1
        result = my_strstr("abababab", "aba")
        assert result == 0
        print("Test multiple occurrences: PASSED")
        return 1

    def test_no_match():
        nonlocal passed, total
        total += 1
        result = my_strstr("hello world", "test")
        assert result == -1
        print("Test no match: PASSED")
        return 1

    def test_unicode_chars():
        nonlocal passed, total
        total += 1
        result = my_strstr("привет мир", "мир")
        assert result == 7
        print("Test unicode chars: PASSED")
        return 1

    try:
        passed += test_empty_needle()
        passed += test_empty_haystack()
        passed += test_needle_longer_than_haystack()
        passed += test_exact_match()
        passed += test_substring_at_start()
        passed += test_substring_at_end()
        passed += test_substring_in_middle()
        passed += test_multiple_occurrences()
        passed += test_no_match()
        passed += test_unicode_chars()
        print(f"\nAll tests passed! {passed}/{total} tests successful.")
    except AssertionError as e:
        print(f"Test failed: {e}")
        print(f"Tests passed: {passed}/{total}")

if __name__ == "__main__":
    run_tests()
