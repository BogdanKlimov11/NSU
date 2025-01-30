def substring_count(string, substring):
    count = 0
    sub_len = len(substring)

    if sub_len == 0:
        return 0  # Если подстрока пустая, возвращаем 0

    i = 0
    while i <= len(string) - sub_len:
        if string[i:i + sub_len] == substring:
            count += 1
            i += 1  # Смещаемся на 1 символ для учета пересечений
        else:
            i += 1

    return count

# Примеры:
print(substring_count("abcabc", "ab"))  # 2
print(substring_count("abcabcd", "d"))  # 1
print(substring_count("abcabcd", "q"))  # 0
print(substring_count("aaaaaa", "aa"))  # 5
