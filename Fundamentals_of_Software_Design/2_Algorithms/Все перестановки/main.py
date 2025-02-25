def permutations(n):
    """Генерирует все перестановки чисел от 0 до N-1."""
    def generate(arr, index, result):
        if index == len(arr):
            result.append(arr[:])  # Добавляем перестановку в результат
            return
        
        for i in range(index, len(arr)):
            arr[index], arr[i] = arr[i], arr[index]  # Меняем элементы местами
            generate(arr, index + 1, result)
            arr[index], arr[i] = arr[i], arr[index]  # Возвращаем массив в исходное состояние

    result = []
    arr = list(range(n))
    generate(arr, 0, result)
    return result

# Тестирование
def test():
    assert permutations(0) == []
    assert permutations(1) == [[0]]
    assert permutations(2) == [[0, 1], [1, 0]]
    assert permutations(3) == [
        [0, 1, 2], [0, 2, 1], [1, 0, 2],
        [1, 2, 0], [2, 0, 1], [2, 1, 0]
    ]
    print("Все тесты пройдены!")

test()
