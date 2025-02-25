import time
import random
from typing import List

# --- Квадратичные сортировки ---
def bubble_sort(arr: List[int]) -> List[int]:
    """Сортировка пузырьком."""
    n = len(arr)
    for i in range(n):
        for j in range(n - 1 - i):
            if arr[j] > arr[j + 1]:
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
    return arr

def insertion_sort(arr: List[int]) -> List[int]:
    """Сортировка вставками."""
    for i in range(1, len(arr)):
        key = arr[i]
        j = i - 1
        while j >= 0 and arr[j] > key:
            arr[j + 1] = arr[j]
            j -= 1
        arr[j + 1] = key
    return arr

def selection_sort(arr: List[int]) -> List[int]:
    """Сортировка выбором."""
    n = len(arr)
    for i in range(n):
        min_idx = i
        for j in range(i + 1, n):
            if arr[j] < arr[min_idx]:
                min_idx = j
        arr[i], arr[min_idx] = arr[min_idx], arr[i]
    return arr

# --- Квази-линейные сортировки ---
def quick_sort(arr: List[int]) -> List[int]:
    """Быстрая сортировка."""
    if len(arr) <= 1:
        return arr
    pivot = arr[len(arr) // 2]
    left = [x for x in arr if x < pivot]
    middle = [x for x in arr if x == pivot]
    right = [x for x in arr if x > pivot]
    return quick_sort(left) + middle + quick_sort(right)

def merge_sort(arr: List[int]) -> List[int]:
    """Сортировка слиянием."""
    if len(arr) <= 1:
        return arr
    mid = len(arr) // 2
    left = merge_sort(arr[:mid])
    right = merge_sort(arr[mid:])
    return merge(left, right)

def merge(left: List[int], right: List[int]) -> List[int]:
    """Слияние массивов для merge_sort."""
    result = []
    i = j = 0
    while i < len(left) and j < len(right):
        if left[i] < right[j]:
            result.append(left[i])
            i += 1
        else:
            result.append(right[j])
            j += 1
    result.extend(left[i:])
    result.extend(right[j:])
    return result

def heap_sort(arr: List[int]) -> List[int]:
    """Пирамидальная сортировка (HeapSort)."""
    def heapify(arr, n, i):
        largest = i
        l = 2 * i + 1
        r = 2 * i + 2
        if l < n and arr[l] > arr[largest]:
            largest = l
        if r < n and arr[r] > arr[largest]:
            largest = r
        if largest != i:
            arr[i], arr[largest] = arr[largest], arr[i]
            heapify(arr, n, largest)

    n = len(arr)
    for i in range(n // 2 - 1, -1, -1):
        heapify(arr, n, i)
    for i in range(n - 1, 0, -1):
        arr[i], arr[0] = arr[0], arr[i]
        heapify(arr, i, 0)
    return arr

# Функция измерения времени работы алгоритма
def measure_time(sort_func, arr: List[int]) -> float:
    start = time.perf_counter()
    sort_func(arr[:])  # Передаем копию массива, чтобы не изменять оригинал
    end = time.perf_counter()
    return end - start

# Тестирование корректности
def test_sorting():
    test_cases = [
        [],
        [42],
        [5, 2, 8, 3, 1],
        [3, 3, 3, 3],
        [-2, -5, 0, 7, 1],
        list(range(100, 0, -1))
    ]
    sorts = [bubble_sort, insertion_sort, selection_sort, quick_sort, merge_sort, heap_sort]
    
    for sort_func in sorts:
        for arr in test_cases:
            assert sort_func(arr[:]) == sorted(arr)
    
    print("Все тесты пройдены!")

# Запуск эксперимента
def benchmark():
    sizes = [100, 200, 400, 800, 1600, 3200]  # Размеры массивов
    sorts = {
        "Bubble Sort": bubble_sort,
        "Insertion Sort": insertion_sort,
        "Selection Sort": selection_sort,
        "Quick Sort": quick_sort,
        "Merge Sort": merge_sort,
        "Heap Sort": heap_sort
    }
    
    print(f"{'Размер':<8} {'Bubble':<10} {'Insert':<10} {'Select':<10} {'Quick':<10} {'Merge':<10} {'Heap':<10}")
    
    for size in sizes:
        arr = [random.randint(-10000, 10000) for _ in range(size)]
        times = {name: measure_time(func, arr) for name, func in sorts.items()}
        
        print(f"{size:<8} {times['Bubble Sort']:.4f} {times['Insertion Sort']:.4f} {times['Selection Sort']:.4f} {times['Quick Sort']:.4f} {times['Merge Sort']:.4f} {times['Heap Sort']:.4f}")

if __name__ == "__main__":
    test_sorting()
    benchmark()
