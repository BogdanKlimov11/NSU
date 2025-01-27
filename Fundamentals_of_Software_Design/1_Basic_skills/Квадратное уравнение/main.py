import math

def solve_quadratic(a, b, c):
    if a == 0:
        # Линейное уравнение bx + c = 0
        if b == 0:
            if c == 0:
                print("Бесконечное количество решений.")
            else:
                print("Решений нет.")
        else:
            x = -c / b
            print(f"Линейное уравнение, одно решение: x = {x:.2f}")
    else:
        # Квадратное уравнение
        discriminant = b ** 2 - 4 * a * c

        if discriminant > 0:
            x1 = (-b + math.sqrt(discriminant)) / (2 * a)
            x2 = (-b - math.sqrt(discriminant)) / (2 * a)
            print(f"Два решения: x1 = {x1:.2f}, x2 = {x2:.2f}")
        elif discriminant == 0:
            x = -b / (2 * a)
            print(f"Одно решение: x = {x:.2f}")
        else:
            print("Действительных решений нет.")

# Основная часть программы
try:
    a = float(input("Введите коэффициент a: "))
    b = float(input("Введите коэффициент b: "))
    c = float(input("Введите коэффициент c: "))

    solve_quadratic(a, b, c)
except ValueError:
    print("Ошибка ввода. Убедитесь, что вы вводите числа.")
