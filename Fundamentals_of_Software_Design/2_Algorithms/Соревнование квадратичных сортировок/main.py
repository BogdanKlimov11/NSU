import random
import time

# --- Сортировка выбором ---
def selection_sort(arr):
    n = len(arr)
    for i in range(n):
        min_idx = i
        for j in range(i + 1, n):
            if arr[j] < arr[min_idx]:
                min_idx = j
        arr[i], arr[min_idx] = arr[min_idx], arr[i]

# --- Сортировка вставками ---
def insertion_sort(arr):
    n = len(arr)
    for i in range(1, n):
        key = arr[i]
        j = i - 1
        while j >= 0 and arr[j] > key:
            arr[j + 1] = arr[j]
            j -= 1
        arr[j + 1] = key

# --- Однонаправленная пузырьковая сортировка ---
def bubble_sort(arr):
    n = len(arr)
    for i in range(n - 1):
        swapped = False
        for j in range(n - 1 - i):
            if arr[j] > arr[j + 1]:
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
                swapped = True
        if not swapped:
            break

# --- Двунаправленная (cocktail) пузырьковая сортировка ---
def cocktail_sort(arr):
    n = len(arr)
    swapped = True
    start, end = 0, n - 1

    while swapped:
        swapped = False
        for i in range(start, end):
            if arr[i] > arr[i + 1]:
                arr[i], arr[i + 1] = arr[i + 1], arr[i]
                swapped = True
        end -= 1

        if not swapped:
            break

        swapped = False
        for i in range(end, start, -1):
            if arr[i] < arr[i - 1]:
                arr[i], arr[i - 1] = arr[i - 1], arr[i]
                swapped = True
        start += 1

# --- Функция тестирования ---
def test_sorts():
    sorts = [selection_sort, insertion_sort, bubble_sort, cocktail_sort]
    test_cases = [
        [],                     # Пустой массив
        [1],                    # Один элемент
        [3, 2, 1],              # Обратный порядок
        [1, 2, 3],              # Уже отсортирован
        [5, 3, 8, 4, 2, 7, 1],  # Случайный порядок
        [1] * 100,              # Все элементы одинаковые
        random.sample(range(1000), 1000)  # 1000 случайных чисел
    ]
    
    for sort in sorts:
        for arr in test_cases:
            copy = arr[:]
            sort(copy)
            assert copy == sorted(arr), f"{sort.__name__} failed!"

    print("Все тесты пройдены!")

test_sorts()

# --- Сравнение времени работы ---
def benchmark():
    sizes = [100, 500, 1000, 2000, 3000]
    sorts = {
        "Selection Sort": selection_sort,
        "Insertion Sort": insertion_sort,
        "Bubble Sort": bubble_sort,
        "Cocktail Sort": cocktail_sort
    }

    for size in sizes:
        print(f"\nРазмер массива: {size}")
        arr = random.sample(range(size * 10), size)

        for name, sort in sorts.items():
            copy = arr[:]
            start = time.perf_counter()
            sort(copy)
            end = time.perf_counter()
            print(f"{name}: {end - start:.5f} сек")

benchmark()

