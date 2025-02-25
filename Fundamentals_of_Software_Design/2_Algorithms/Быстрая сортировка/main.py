import random

def quick_sort(arr):
    """Функция сортирует массив in-place с использованием быстрой сортировки."""
    def partition(low, high):
        pivot_index = random.randint(low, high)
        arr[pivot_index], arr[high] = arr[high], arr[pivot_index]  # Меняем местами опорный и последний элемент
        pivot = arr[high]
        i = low
        for j in range(low, high):
            if arr[j] < pivot:
                arr[i], arr[j] = arr[j], arr[i]
                i += 1
        arr[i], arr[high] = arr[high], arr[i]
        return i

    def quick_sort_recursive(low, high):
        if low < high:
            pi = partition(low, high)
            quick_sort_recursive(low, pi - 1)
            quick_sort_recursive(pi + 1, high)

    if len(arr) > 1:
        quick_sort_recursive(0, len(arr) - 1)

# Тестирование
def test():
    # Краевые случаи
    arr1 = []
    quick_sort(arr1)
    assert arr1 == []

    arr2 = [5]
    quick_sort(arr2)
    assert arr2 == [5]

    arr3 = [3, 1, 4, 1, 5, 9, 2, 6]
    expected3 = sorted(arr3)
    quick_sort(arr3)
    assert arr3 == expected3

    arr4 = [10, 20, 30, 40, 50]
    expected4 = [10, 20, 30, 40, 50]
    quick_sort(arr4)
    assert arr4 == expected4

    arr5 = [5, 5, 5, 5, 5]
    expected5 = [5, 5, 5, 5, 5]
    quick_sort(arr5)
    assert arr5 == expected5

    # Тест случайных массивов
    for _ in range(10):
        arr = [random.randint(0, 100) for _ in range(random.randint(0, 20))]
        expected = sorted(arr)
        quick_sort(arr)
        assert arr == expected

    print("Все тесты пройдены!")

test()
