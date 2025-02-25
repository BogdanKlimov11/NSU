def compute_lps(needle):
    """Вычисляет массив LPS (longest prefix suffix) для подстроки needle."""
    lps = [0] * len(needle)
    length = 0
    i = 1

    while i < len(needle):
        if needle[i] == needle[length]:
            length += 1
            lps[i] = length
            i += 1
        elif length:
            length = lps[length - 1]
        else:
            lps[i] = 0
            i += 1
    return lps

def my_strstr(haystack, needle):
    """Алгоритм Кнута-Морриса-Пратта (КМП) для поиска подстроки."""
    if not needle:
        return 0

    n, m = len(haystack), len(needle)
    lps = compute_lps(needle)
    i = j = 0  # Индексы для haystack и needle

    while i < n:
        if haystack[i] == needle[j]:
            i += 1
            j += 1
        if j == m:
            return i - j  # Найдено вхождение
        elif i < n and haystack[i] != needle[j]:
            j = lps[j - 1] if j else 0
            i += not j  # Увеличиваем i, если j == 0
    return None  # Не найдено

# --- Тесты ---
def test_my_strstr():
    assert my_strstr("hello", "ll") == 2
    assert my_strstr("abcdef", "cd") == 2
    assert my_strstr("aaaaa", "bba") is None
    assert my_strstr("mississippi", "issi") == 1
    assert my_strstr("abc", "") == 0
    assert my_strstr("", "abc") is None
    assert my_strstr("abc", "abc") == 0
    print("Все тесты пройдены!")

test_my_strstr()
