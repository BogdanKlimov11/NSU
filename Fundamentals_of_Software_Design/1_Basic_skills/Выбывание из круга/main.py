def josephus_problem(n, k):
    # Список пользователей
    users = list(range(1, n + 1))
    index = 0  # Индекс для отсчета
    
    # Пока в списке больше одного пользователя
    while len(users) > 1:
        # Вычисляем индекс для выбывающего пользователя
        index = (index + k - 1) % len(users)
        # Выводим номер выбывающего
        print(users.pop(index))

    # Последний оставшийся пользователь
    print(users[0])

if __name__ == '__main__':
    n = int(input("Введите количество пользователей (N): "))
    k = int(input("Введите шаг выбывания (K): "))
    
    josephus_problem(n, k)
