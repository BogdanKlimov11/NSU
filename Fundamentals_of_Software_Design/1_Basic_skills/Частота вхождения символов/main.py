def char_frequency(s):
    freq = {}
    for char in s:
        if char in freq:
            freq[char] += 1
        else:
            freq[char] = 1
    return freq


# Пример использования
s = "aaaabccc"
freq = char_frequency(s)

for char, count in freq.items():
    print(f'"{char}" — {count} раз(а).')
