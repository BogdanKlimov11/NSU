import random

def shuffle_array(arr):
    """
    Перемешивает элементы массива случайным образом.

    :param arr: Список элементов для перемешивания.
    """
    n = len(arr)
    for i in range(n - 1, 0, -1):
        # Генерируем случайный индекс от 0 до i включительно
        j = random.randint(0, i)
        # Меняем местами элементы arr[i] и arr[j]
        arr[i], arr[j] = arr[j], arr[i]

# Пример использования
if __name__ == "__main__":
    array = [1, 2, 3, 4, 5]
    print("Исходный массив:", array)
    shuffle_array(array)
    print("Перемешанный массив:", array)
