def strtrim(string):
    """
    Удаляет пробелы в начале и конце строки.
    Изменяет строку, если она передана как изменяемый объект (например, list).
    """
    if not string:
        return ""

    # Удаление пробелов в начале
    start = 0
    while start < len(string) and string[start].isspace():
        start += 1

    # Удаление пробелов в конце
    end = len(string) - 1
    while end >= start and string[end].isspace():
        end -= 1

    # Возврат среза строки
    return string[start:end + 1]


# Пример использования
buf = "   abc   "
print(f"<{buf}>")  # <   abc   >
trimmed = strtrim(buf)
print(f"<{trimmed}>")  # <abc>
