from typing import List

def z_function(s: str) -> List[int]:
    n = len(s)
    z = [0] * n
    l, r = 0, 0
    for i in range(1, n):
        if i <= r:
            z[i] = min(r - i + 1, z[i - l])
        while i + z[i] < n and s[z[i]] == s[i + z[i]]:
            z[i] += 1
        if i + z[i] - 1 > r:
            l, r = i, i + z[i] - 1
    return z

def substring_search(text: str, pattern: str) -> List[int]:
    combined = pattern + "#" + text
    z = z_function(combined)
    pattern_len = len(pattern)
    result = [i - pattern_len - 1 for i in range(len(z)) if z[i] == pattern_len]
    return result

# Тесты
def test():
    assert substring_search("ababcababc", "abc") == [2, 7]
    assert substring_search("aaaaa", "aa") == [0, 1, 2, 3]
    assert substring_search("abcdef", "xyz") == []
    assert substring_search("abababab", "aba") == [0, 2, 4]
    assert substring_search("", "a") == []
    assert substring_search("a", "a") == [0]
    print("Все тесты пройдены!")

test()
