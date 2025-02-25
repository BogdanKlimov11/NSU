def heapify(arr, n, i):
    """Приводит поддерево с корнем в i к виду кучи (max-heap)."""
    largest = i
    left = 2 * i + 1
    right = 2 * i + 2

    if left < n and arr[left] > arr[largest]:
        largest = left

    if right < n and arr[right] > arr[largest]:
        largest = right

    if largest != i:
        arr[i], arr[largest] = arr[largest], arr[i]  # Меняем местами
        heapify(arr, n, largest)

def heap_sort(arr):
    """Реализует пирамидальную сортировку."""
    n = len(arr)

    # Строим кучу (max-heap)
    for i in range(n // 2 - 1, -1, -1):
        heapify(arr, n, i)

    # Извлекаем элементы из кучи по одному
    for i in range(n - 1, 0, -1):
        arr[i], arr[0] = arr[0], arr[i]  # Меняем корень с последним элементом
        heapify(arr, i, 0)  # Восстанавливаем кучу для уменьшенного массива

# Тестирование
def test():
    # Краевые случаи
    arr1 = []
    heap_sort(arr1)
    assert arr1 == []

    arr2 = [5]
    heap_sort(arr2)
    assert arr2 == [5]

    arr3 = [3, 1, 4, 1, 5, 9, 2, 6]
    expected3 = sorted(arr3)
    heap_sort(arr3)
    assert arr3 == expected3

    arr4 = [10, 20, 30, 40, 50]
    expected4 = [10, 20, 30, 40, 50]
    heap_sort(arr4)
    assert arr4 == expected4

    arr5 = [5, 5, 5, 5, 5]
    expected5 = [5, 5, 5, 5, 5]
    heap_sort(arr5)
    assert arr5 == expected5

    # Тест случайных массивов
    for _ in range(10):
        arr = [random.randint(0, 100) for _ in range(random.randint(0, 20))]
        expected = sorted(arr)
        heap_sort(arr)
        assert arr == expected

    print("Все тесты пройдены!")

test()
