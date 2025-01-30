import random

def is_lucky_ticket(ticket):
    sum1 = sum(int(ticket[i]) for i in range(16))
    sum2 = sum(int(ticket[i]) for i in range(16, 32))
    return sum1 == sum2

def generate_ticket():
    # Генерируем случайный 32-значный номер билета
    return ''.join(str(random.randint(0, 9)) for _ in range(32))

def main():
    total_tickets = 1000000  # Количество билетов для проверки
    lucky_count = 0  # Количество счастливых билетов

    for _ in range(total_tickets):
        ticket = generate_ticket()
        if is_lucky_ticket(ticket):
            lucky_count += 1

    # Вычисляем долю счастливых билетов
    lucky_percentage = (lucky_count / total_tickets) * 100

    print(f"Доля счастливых билетов: {lucky_percentage:.6f}%")

if __name__ == "__main__":
    main()
