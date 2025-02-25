import random
import time

# Размер массива
SIZE = 1000000

# Сортировка вставками
def insertion_sort(arr):
    for i in range(1, len(arr)):
        key = arr[i]
        j = i - 1
        while j >= 0 and arr[j] > key:
            arr[j + 1] = arr[j]
            j -= 1
        arr[j + 1] = key

# Быстрая сортировка
def quick_sort(arr):
    if len(arr) <= 1:
        return arr
    pivot = arr[len(arr) // 2]
    left = [x for x in arr if x < pivot]
    middle = [x for x in arr if x == pivot]
    right = [x for x in arr if x > pivot]
    return quick_sort(left) + middle + quick_sort(right)

# Поразрядная сортировка
def counting_sort_by_byte(arr, exp):
    count = [0] * 256
    output = [0] * len(arr)

    # Подсчет вхождений каждого байта
    for num in arr:
        index = (num // exp) % 256
        count[index] += 1

    # Изменяем count[i] так, чтобы оно содержало позиции элементов в output
    for i in range(1, 256):
        count[i] += count[i - 1]

    # Строим отсортированный массив
    for i in range(len(arr) - 1, -1, -1):
        num = arr[i]
        index = (num // exp) % 256
        output[count[index] - 1] = num
        count[index] -= 1

    # Копируем отсортированные элементы обратно в оригинальный массив
    for i in range(len(arr)):
        arr[i] = output[i]

def radix_sort(arr):
    # Проходим по каждому байту числа, начиная с младшего
    max_num = max(arr)
    exp = 1
    while max_num // exp > 0:
        counting_sort_by_byte(arr, exp)
        exp *= 256

# Генерация случайного массива
def generate_random_array(n):
    return [random.randint(0, 1000000) for _ in range(n)]

# Тестирование времени выполнения сортировок
def test_sorting_algorithms():
    arr1 = generate_random_array(SIZE)
    arr2 = arr1[:]
    arr3 = arr1[:]
    arr4 = arr1[:]

    # Сортировка вставками
    start = time.perf_counter()
    insertion_sort(arr1)
    end = time.perf_counter()
    print(f"Сортировка вставками: {end - start:.4f} секунд")

    # Быстрая сортировка
    start = time.perf_counter()
    arr2 = quick_sort(arr2)
    end = time.perf_counter()
    print(f"Быстрая сортировка: {end - start:.4f} секунд")

    # Поразрядная сортировка
    start = time.perf_counter()
    radix_sort(arr3)
    end = time.perf_counter()
    print(f"Поразрядная сортировка: {end - start:.4f} секунд")

    # Встроенная сортировка (sorted)
    start = time.perf_counter()
    arr4 = sorted(arr4)
    end = time.perf_counter()
    print(f"Встроенная сортировка (sorted): {end - start:.4f} секунд")

# Запуск тестов
test_sorting_algorithms()
