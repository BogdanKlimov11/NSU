import numpy as np
import matplotlib.pyplot as plt
from scipy.sparse import diags
from scipy.sparse.linalg import spsolve

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Константы
T0 = 273.15 # 0°C в Кельвинах
K = 0.025 # Теплопроводность, Вт/(м·К)
RHO = 1.2 # Плотность, кг/м³
C = 1005 # Теплоемкость, Дж/(кг·К)
ALPHA = K / (RHO * C)
Q = 1000 # Источник тепла, Вт/м³

# Параметры сетки
L = 1.0 # Длина стержня, м
NX = 100 # Количество узлов
DX = L / NX
X = np.linspace(0, L, NX + 1)

# Временные параметры
TOTAL_TIME = 2.0
DT = 0.1 * DX**2 / (2 * ALPHA)  # Условие устойчивости для явной схемы
NT = int(TOTAL_TIME / DT) + 1

# Граничные условия (в Кельвинах)
def t_left(t):
    return max(283.15, 293.15 - 5 * t)  # Минимум 10°C

def t_right(t):
    return max(263.15, 273.15 - 5 * t)  # Минимум -10°C

# Точные решения
def exact_solution_no_source(x):
    return 293.15 * (1 - x) + 273.15 * x  # Линейное распределение

def exact_solution_with_source(x):
    return (293.15 * (1 - x) + 273.15 * x) + (Q * L**2 / (2 * K)) * (x - x**2)

# Task 1a: Стационарное решение без источника (метод Якоби)
def task_1a():
    setup_plot()
    t = np.linspace(t_left(0), t_right(0), NX + 1)
    tolerance = 1e-6
    
    for _ in range(10000):
        t_old = t.copy()
        for i in range(1, NX):
            t[i] = 0.5 * (t[i-1] + t[i+1])
        if np.max(np.abs(t - t_old)) < tolerance:
            break
    
    plt.figure(figsize=(10, 5))
    plt.plot(X, t - T0, label='Численное решение')
    plt.plot(X, exact_solution_no_source(X) - T0, '--', label='Точное решение')
    plt.title('Стационарное решение без источника (Task 1a)')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    return X, t

# Task 1b: Нестационарный случай без источника (явная схема)
def task_1b():
    setup_plot()
    t = np.linspace(t_left(0), t_right(0), NX + 1)
    times = np.linspace(0, TOTAL_TIME, NT)
    temps = np.zeros((NT, NX + 1))
    temps[0] = t.copy()
    r = ALPHA * DT / DX**2
    
    for n in range(1, NT):
        t_new = temps[n-1].copy()
        t_new[0] = t_left(times[n])
        t_new[-1] = t_right(times[n])
        
        for i in range(1, NX):
            t_new[i] = temps[n-1, i] + r * (temps[n-1, i-1] - 2*temps[n-1, i] + temps[n-1, i+1])
        
        temps[n] = t_new
    
    # Визуализация
    plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, NT-1, 5, dtype=int):
        plt.plot(X, temps[idx] - T0, label=f't={times[idx]:.2f} с')
    plt.title('Нестационарное решение без источника (Task 1b)')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    return X, times, temps

# Task 1c: Нестационарный случай с источником (явная схема)
def task_1c():
    setup_plot()
    t = np.linspace(t_left(0), t_right(0), NX + 1)
    times = np.linspace(0, TOTAL_TIME, NT)
    temps = np.zeros((NT, NX + 1))
    temps[0] = t.copy()
    r = ALPHA * DT / DX**2
    q_term = Q * DT / (RHO * C)
    
    for n in range(1, NT):
        t_new = temps[n-1].copy()
        t_new[0] = t_left(times[n])
        t_new[-1] = t_right(times[n])
        
        for i in range(1, NX):
            t_new[i] = temps[n-1, i] + r * (temps[n-1, i-1] - 2*temps[n-1, i] + temps[n-1, i+1]) + q_term
        
        temps[n] = t_new
    
    # Визуализация
    plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, NT-1, 5, dtype=int):
        plt.plot(X, temps[idx] - T0, label=f't={times[idx]:.2f} с')
    plt.title('Нестационарное решение с источником (Task 1c)')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    return X, times, temps

# Task 2a: Стационарное решение без источника (неявная схема)
def task_2a():
    setup_plot()
    t = np.linspace(t_left(0), t_right(0), NX + 1)
    times = np.linspace(0, TOTAL_TIME, NT)
    temps = np.zeros((NT, NX + 1))
    temps[0] = t.copy()
    r = ALPHA * DT / DX**2
    
    # Построение матрицы
    diagonals = [[-r]*(NX-2), [1+2*r]*(NX-1), [-r]*(NX-2)]
    A = diags(diagonals, [-1, 0, 1], format='csc')
    
    for n in range(1, NT):
        b = temps[n-1, 1:-1].copy()
        b[0] += r * t_left(0)
        b[-1] += r * t_right(0)
        
        temps[n, 1:-1] = spsolve(A, b)
        temps[n, 0] = t_left(0)
        temps[n, -1] = t_right(0)
    
    # Визуализация
    plt.figure(figsize=(10, 5))
    plt.plot(X, temps[-1] - T0, label='Численное решение')
    plt.plot(X, exact_solution_no_source(X) - T0, '--', label='Точное решение')
    plt.title('Стационарное решение (неявная схема, Task 2a)')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    return X, times, temps

# Task 2b: Нестационарный случай без источника (неявная схема)
def task_2b():
    setup_plot()
    t = np.linspace(t_left(0), t_right(0), NX + 1)
    times = np.linspace(0, TOTAL_TIME, NT)
    temps = np.zeros((NT, NX + 1))
    temps[0] = t.copy()
    r = ALPHA * DT / DX**2
    
    diagonals = [[-r]*(NX-2), [1+2*r]*(NX-1), [-r]*(NX-2)]
    A = diags(diagonals, [-1, 0, 1], format='csc')
    
    for n in range(1, NT):
        current_time = times[n]
        b = temps[n-1, 1:-1].copy()
        b[0] += r * t_left(current_time)
        b[-1] += r * t_right(current_time)
        
        temps[n, 1:-1] = spsolve(A, b)
        temps[n, 0] = t_left(current_time)
        temps[n, -1] = t_right(current_time)
    
    # Визуализация
    plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, NT-1, 5, dtype=int):
        plt.plot(X, temps[idx] - T0, label=f't={times[idx]:.2f} с')
    plt.title('Нестационарное решение (неявная схема, Task 2b)')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    return X, times, temps

# Task 2c: Нестационарный случай с источником (неявная схема)
def task_2c():
    setup_plot()
    t = np.linspace(t_left(0), t_right(0), NX + 1)
    times = np.linspace(0, TOTAL_TIME, NT)
    temps = np.zeros((NT, NX + 1))
    temps[0] = t.copy()
    r = ALPHA * DT / DX**2
    q_term = Q * DT / (RHO * C)
    
    diagonals = [[-r]*(NX-2), [1+2*r]*(NX-1), [-r]*(NX-2)]
    A = diags(diagonals, [-1, 0, 1], format='csc')
    
    for n in range(1, NT):
        current_time = times[n]
        b = temps[n-1, 1:-1].copy() + q_term
        b[0] += r * t_left(current_time)
        b[-1] += r * t_right(current_time)
        
        temps[n, 1:-1] = spsolve(A, b)
        temps[n, 0] = t_left(current_time)
        temps[n, -1] = t_right(current_time)
    
    # Визуализация
    plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, NT-1, 5, dtype=int):
        plt.plot(X, temps[idx] - T0, label=f't={times[idx]:.2f} с')
    plt.title('Нестационарное решение с источником (неявная схема, Task 2c)')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    return X, times, temps

if __name__ == "__main__":
    setup_plot()
    task_1a()
    task_1b()
    task_1c()
    task_2a()
    task_2b()
    task_2c()