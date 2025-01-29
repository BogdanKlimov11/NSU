def is_leap_year(year):
    return (year % 4 == 0 and year % 100 != 0) or (year % 400 == 0)

def day_number(d, m, y):
    if m < 1 or m > 12 or d < 1:  # Проверка корректности месяца и дня
        return -1

    days_in_months = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    if is_leap_year(y):
        days_in_months[1] = 29  # Коррекция февраля для високосного года

    if d > days_in_months[m - 1]:  # Проверка корректности дня
        return -1

    return sum(days_in_months[:m - 1]) + d

# Пример использования:
d, m, y = map(int, input("Введите день, месяц и год: ").split())
result = day_number(d, m, y)

if result == -1:
    print("Некорректная дата")
else:
    print(f"Номер дня в году: {result}")
