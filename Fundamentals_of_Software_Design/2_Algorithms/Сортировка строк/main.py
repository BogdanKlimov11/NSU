def counting_sort(arr, char_index):
    # Массив для подсчета символов (от 0 до 255 для ASCII)
    count = [0] * 256
    # Массив для результата сортировки
    output = [''] * len(arr)
    
    # Подсчитаем количество каждого символа в заданном разряде (char_index)
    for string in arr:
        count[ord(string[char_index])] += 1
    
    # Преобразуем count в массив позиций
    for i in range(1, 256):
        count[i] += count[i - 1]
    
    # Строим отсортированный результат
    for string in reversed(arr):  # для стабильности сортировки
        output[count[ord(string[char_index])] - 1] = string
        count[ord(string[char_index])] -= 1
    
    return output

def radix_sort_strings(arr):
    if len(arr) == 0:
        return arr
    
    # Определим максимальную длину строки
    max_len = max(len(s) for s in arr)
    
    # Проводим сортировку по каждому символу от старшего к младшему
    for char_index in range(max_len - 1, -1, -1):
        arr = counting_sort(arr, char_index)
    
    return arr

# Тестирование:
strings = ["apple", "banana", "grape", "cherry", "date"]
sorted_strings = radix_sort_strings(strings)

print("Отсортированные строки:", sorted_strings)
