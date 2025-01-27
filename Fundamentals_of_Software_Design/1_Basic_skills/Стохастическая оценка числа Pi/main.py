import random

def estimate_pi(num_points):
    inside_circle = 0

    for _ in range(num_points):
        # Генерируем случайные координаты (x, y) в диапазоне [0, 1]
        x = random.uniform(0, 1)
        y = random.uniform(0, 1)
        
        # Проверяем, попадает ли точка в четверть окружности
        if x**2 + y**2 <= 1:
            inside_circle += 1

    # Площадь четверти круга приближенно равна отношению точек внутри круга к общему числу точек
    # Умножаем на 4 для получения площади полного круга
    return (inside_circle / num_points) * 4

# Ввод количества точек
num_points = int(input("Введите количество точек для метода Монте-Карло: "))

# Оценка числа π
pi_estimate = estimate_pi(num_points)
print(f"Оценка числа π: {pi_estimate}")
