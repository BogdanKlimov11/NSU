def preprocess_bad_character_rule(needle):
    """Предобработка для правила плохих символов"""
    bad_char_table = {}
    m = len(needle)

    for i in range(m):
        bad_char_table[needle[i]] = i  # Индекс последнего появления символа в needle

    return bad_char_table

def my_strstr(haystack, needle):
    n = len(haystack)
    m = len(needle)

    if m == 0:
        return 0  # Пустая строка всегда найдется

    bad_char_table = preprocess_bad_character_rule(needle)

    i = 0
    while i <= n - m:
        j = m - 1
        while j >= 0 and haystack[i + j] == needle[j]:
            j -= 1

        if j < 0:
            return i  # Возвращаем индекс начала совпадения

        # Сдвигаем на основе таблицы плохих символов
        shift = j - bad_char_table.get(haystack[i + j], -1)
        i += max(shift, 1)

    return -1  # Если подстрока не найдена

# Тестирование
haystack = "this is a simple example"
needle = "simple"

result = my_strstr(haystack, needle)

if result != -1:
    print(f"Подстрока найдена на позиции: {result}")
else:
    print("Подстрока не найдена")
