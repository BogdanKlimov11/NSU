import random

def test_random_distribution(n):
    count = {i: 0 for i in range(101)}  # Словарь для подсчета частоты выпадений чисел

    for _ in range(n):
        num = random.randint(0, 100)
        count[num] += 1

    # Вывод статистики
    for number in range(101):
        print(f"{number}: {count[number]} раз")

if __name__ == "__main__":
    n = int(input("Введите количество случайных чисел для генерации: "))
    test_random_distribution(n)
