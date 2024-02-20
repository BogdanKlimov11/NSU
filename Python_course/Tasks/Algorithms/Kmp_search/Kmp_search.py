# Поиск подстроки в строке алгоритмом Кнута-Морриса-Пратта
def kmp_search(haystack, needle):
    m = len(needle)
    n = len(haystack)
    if m == 0:
        return None
    # Создаем префикс-функцию для иглы
    prefix = compute_prefix(needle)
    # Начинаем поиск с начала текста и иглы
    i = 0
    j = 0
    while i < n:
        if haystack[i] == needle[j]:
            i += 1
            j += 1
            if j == m:
                return i - j
        elif j > 0:
            j = prefix[j - 1]
        else:
            i += 1
    return None

# Префикс-функция для иглы
def compute_prefix(needle):
    m = len(needle)
    prefix = [0] * m
    length = 0
    i = 1
    while i < m:
        if needle[i] == needle[length]:
            length += 1
            prefix[i] = length
            i += 1
        else:
            if length != 0:
                length = prefix[length - 1]
            else:
                prefix[i] = 0
                i += 1
    return prefix