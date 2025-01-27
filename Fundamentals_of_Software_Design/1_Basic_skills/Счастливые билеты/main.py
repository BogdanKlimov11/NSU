def count_lucky_tickets():
    total_tickets = 0  # Общее количество билетов
    lucky_tickets = 0  # Количество счастливых билетов

    for ticket in range(1000000):  # Перебираем все 6-значные номера билетов
        # Разделяем номер билета на цифры
        digits = [int(d) for d in f"{ticket:06}"]  # Преобразуем число в 6-значный формат
        first_half = sum(digits[:3])  # Сумма первых трёх цифр
        second_half = sum(digits[3:])  # Сумма последних трёх цифр

        if first_half == second_half:  # Проверяем на счастливость
            lucky_tickets += 1

        total_tickets += 1

    # Вычисляем процент счастливых билетов
    percentage = (lucky_tickets / total_tickets) * 100

    print(f"Общее количество билетов: {total_tickets}")
    print(f"Количество счастливых билетов: {lucky_tickets}")
    print(f"Процент счастливых билетов: {percentage:.2f}%")

# Запуск программы
count_lucky_tickets()
