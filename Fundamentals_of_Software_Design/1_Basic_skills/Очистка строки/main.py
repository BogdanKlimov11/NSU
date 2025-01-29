def strclear(string):
    i = 0
    result = []
    space_found = False  # Флаг, чтобы отслеживать, если был пробел подряд

    # Перебираем строку
    while i < len(string):
        if string[i].isspace():
            if not space_found:  # Если пробела еще не было, то ставим один
                result.append(' ')
                space_found = True
        else:
            result.append(string[i])  # Просто добавляем символ
            space_found = False  # Сброс флага
        i += 1

    return ''.join(result)

# Пример использования
input_str = "ab    cd"
print(f"<{input_str}>")  # <ab    cd>
output_str = strclear(input_str)
print(f"<{output_str}>")  # <ab cd>
