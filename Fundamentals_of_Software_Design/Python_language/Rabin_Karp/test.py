from rabin_karp import my_strstr

def run_tests():
    passed = 0
    total = 0

    def test_empty_needle():
        nonlocal passed, total
        total += 1
        assert my_strstr("any string", "") == 0
        print("Test empty needle: PASSED")
        return 1

    def test_empty_haystack():
        nonlocal passed, total
        total += 1
        assert my_strstr("", "needle") == -1
        print("Test empty haystack: PASSED")
        return 1

    def test_needle_longer():
        nonlocal passed, total
        total += 1
        assert my_strstr("short", "very long needle") == -1
        print("Test needle longer: PASSED")
        return 1

    def test_exact_match():
        nonlocal passed, total
        total += 1
        assert my_strstr("hello world", "hello world") == 0
        print("Test exact match: PASSED")
        return 1

    def test_start():
        nonlocal passed, total
        total += 1
        assert my_strstr("hello world", "hello") == 0
        print("Test start: PASSED")
        return 1

    def test_end():
        nonlocal passed, total
        total += 1
        assert my_strstr("hello world", "world") == 6
        print("Test end: PASSED")
        return 1

    def test_middle():
        nonlocal passed, total
        total += 1
        assert my_strstr("hello beautiful world", "beautiful") == 6
        print("Test middle: PASSED")
        return 1

    def test_multiple():
        nonlocal passed, total
        total += 1
        assert my_strstr("abababab", "aba") == 0
        print("Test multiple: PASSED")
        return 1

    def test_no_match():
        nonlocal passed, total
        total += 1
        assert my_strstr("hello world", "test") == -1
        print("Test no match: PASSED")
        return 1

    def test_collision():
        nonlocal passed, total
        total += 1
        assert my_strstr("abcba", "ba") == 3
        print("Test collision: PASSED")
        return 1

    def test_unicode():
        nonlocal passed, total
        total += 1
        assert my_strstr("привет мир", "мир") == 7
        print("Test unicode: PASSED")
        return 1

    try:
        passed += test_empty_needle()
        passed += test_empty_haystack()
        passed += test_needle_longer()
        passed += test_exact_match()
        passed += test_start()
        passed += test_end()
        passed += test_middle()
        passed += test_multiple()
        passed += test_no_match()
        passed += test_collision()
        passed += test_unicode()
        print(f"\nAll tests passed! {passed}/{total} tests successful.")
    except AssertionError as e:
        print(f"Test failed: {e}")
        print(f"Tests passed: {passed}/{total}")

if __name__ == "__main__":
    run_tests()
