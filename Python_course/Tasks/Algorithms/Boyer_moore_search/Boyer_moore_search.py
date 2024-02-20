# Поиск подстроки в строке алгоритмом Боуэра-Мура
def boyer_moore_search(haystack, needle):
    m = len(needle)
    n = len(haystack)
    if m == 0:
        return None
    # Создаем таблицу сдвигов для каждого символа в игле
    shift_table = {}
    for i in range(m):
        shift_table[needle[i]] = m - i - 1
    # Начинаем поиск с конца иглы
    i = m - 1
    while i < n:
        j = m - 1
        while j >= 0 and haystack[i] == needle[j]:
            i -= 1
            j -= 1
        if j == -1:
            return i + 1
        # Вычисляем сдвиг на основе таблицы сдвигов
        if haystack[i] in shift_table:
            i += shift_table[haystack[i]]
        else:
            i += m
    return None