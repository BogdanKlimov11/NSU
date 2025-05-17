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
temp_0c = 273.15
thermal_conductivity = 0.025
density = 1.2
specific_heat = 1005
alpha = thermal_conductivity / (density * specific_heat)
heat_source = 1000

# Параметры сетки
rod_length = 1.0
num_nodes = 100
dx = rod_length / num_nodes
x_array = np.linspace(0, rod_length, num_nodes + 1)

# Временные параметры
total_time = 2.0
dt = 0.1 * dx**2 / (2 * alpha)  # Условие устойчивости для явной схемы
num_time_steps = int(total_time / dt) + 1

# Граничные условия (в Кельвинах)
def temp_left(t):
    return max(283.15, 293.15 - 5 * t)  # Минимум 10°C

def temp_right(t):
    return max(263.15, 273.15 - 5 * t)  # Минимум -10°C

# Точные решения
def exact_solution_no_source(x):
    return 293.15 * (1 - x) + 273.15 * x

def exact_solution_with_source(x):
    return (293.15 * (1 - x) + 273.15 * x) + (heat_source * rod_length**2 / (2 * thermal_conductivity)) * (x - x**2)

# Task 1a: Стационарное решение без источника (метод Якоби)
def task_1a():
    print("Plotting Task 1a: Stationary solution without source (Jacobi method)")
    setup_plot()
    temp = np.linspace(temp_left(0), temp_right(0), num_nodes + 1)
    tolerance = 1e-6
    for _ in range(10000):
        temp_old = temp.copy()
        for i in range(1, num_nodes):
            temp[i] = 0.5 * (temp[i-1] + temp[i+1])
        if np.max(np.abs(temp - temp_old)) < tolerance:
            break
    fig = plt.figure(figsize=(10, 5))
    plt.plot(x_array, temp - temp_0c, label='Численное решение')
    plt.plot(x_array, exact_solution_no_source(x_array) - temp_0c, '--', label='Точное решение')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.title('Стационарное решение без источника (метод Якоби)')
    plt.grid(True)
    plt.legend()
    fig.canvas.manager.set_window_title('Task 1a: Stationary solution without source (Jacobi method)')
    plt.savefig('task_1a.png')
    plt.show()
    plt.close()
    return x_array, temp

# Task 1b: Нестационарный случай без источника (явная схема)
def task_1b():
    print("Plotting Task 1b: Transient solution without source (explicit scheme)")
    setup_plot()
    temp = np.linspace(temp_left(0), temp_right(0), num_nodes + 1)
    times = np.linspace(0, total_time, num_time_steps)
    temps = np.zeros((num_time_steps, num_nodes + 1))
    temps[0] = temp.copy()
    r = alpha * dt / dx**2
    for n in range(1, num_time_steps):
        temp_new = temps[n-1].copy()
        temp_new[0] = temp_left(times[n])
        temp_new[-1] = temp_right(times[n])
        for i in range(1, num_nodes):
            temp_new[i] = temps[n-1, i] + r * (temps[n-1, i-1] - 2*temps[n-1, i] + temps[n-1, i+1])
        temps[n] = temp_new
    fig = plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, num_time_steps-1, 5, dtype=int):
        plt.plot(x_array, temps[idx] - temp_0c, label=f't={times[idx]:.2f} с')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.title('Нестационарное решение без источника (явная схема)')
    plt.grid(True)
    plt.legend()
    fig.canvas.manager.set_window_title('Task 1b: Transient solution without source (explicit scheme)')
    plt.savefig('task_1b.png')
    plt.show()
    plt.close()
    return x_array, times, temps

# Task 1c: Нестационарный случай с источником (явная схема)
def task_1c():
    print("Plotting Task 1c: Transient solution with source (explicit scheme)")
    setup_plot()
    temp = np.linspace(temp_left(0), temp_right(0), num_nodes + 1)
    times = np.linspace(0, total_time, num_time_steps)
    temps = np.zeros((num_time_steps, num_nodes + 1))
    temps[0] = temp.copy()
    r = alpha * dt / dx**2
    q_term = heat_source * dt / (density * specific_heat)
    for n in range(1, num_time_steps):
        temp_new = temps[n-1].copy()
        temp_new[0] = temp_left(times[n])
        temp_new[-1] = temp_right(times[n])
        for i in range(1, num_nodes):
            temp_new[i] = temps[n-1, i] + r * (temps[n-1, i-1] - 2*temps[n-1, i] + temps[n-1, i+1]) + q_term
        temps[n] = temp_new
    fig = plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, num_time_steps-1, 5, dtype=int):
        plt.plot(x_array, temps[idx] - temp_0c, label=f't={times[idx]:.2f} с')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.title('Нестационарное решение с источником (явная схема)')
    plt.grid(True)
    plt.legend()
    fig.canvas.manager.set_window_title('Task 1c: Transient solution with source (explicit scheme)')
    plt.savefig('task_1c.png')
    plt.show()
    plt.close()
    return x_array, times, temps

# Task 2a: Стационарное решение без источника (неявная схема)
def task_2a():
    print("Plotting Task 2a: Stationary solution without source (implicit scheme)")
    setup_plot()
    temp = np.linspace(temp_left(0), temp_right(0), num_nodes + 1)
    times = np.linspace(0, total_time, num_time_steps)
    temps = np.zeros((num_time_steps, num_nodes + 1))
    temps[0] = temp.copy()
    r = alpha * dt / dx**2
    diagonals = [[-r]*(num_nodes-2), [1+2*r]*(num_nodes-1), [-r]*(num_nodes-2)]
    matrix_a = diags(diagonals, [-1, 0, 1], format='csc')
    for n in range(1, num_time_steps):
        b = temps[n-1, 1:-1].copy()
        b[0] += r * temp_left(0)
        b[-1] += r * temp_right(0)
        temps[n, 1:-1] = spsolve(matrix_a, b)
        temps[n, 0] = temp_left(0)
        temps[n, -1] = temp_right(0)
    fig = plt.figure(figsize=(10, 5))
    plt.plot(x_array, temps[-1] - temp_0c, label='Численное решение')
    plt.plot(x_array, exact_solution_no_source(x_array) - temp_0c, '--', label='Точное решение')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.title('Стационарное решение без источника (неявная схема)')
    plt.grid(True)
    plt.legend()
    fig.canvas.manager.set_window_title('Task 2a: Stationary solution without source (implicit scheme)')
    plt.savefig('task_2a.png')
    plt.show()
    plt.close()
    return x_array, times, temps

# Task 2b: Нестационарный случай без источника (неявная схема)
def task_2b():
    print("Plotting Task 2b: Transient solution without source (implicit scheme)")
    setup_plot()
    temp = np.linspace(temp_left(0), temp_right(0), num_nodes + 1)
    times = np.linspace(0, total_time, num_time_steps)
    temps = np.zeros((num_time_steps, num_nodes + 1))
    temps[0] = temp.copy()
    r = alpha * dt / dx**2
    diagonals = [[-r]*(num_nodes-2), [1+2*r]*(num_nodes-1), [-r]*(num_nodes-2)]
    matrix_a = diags(diagonals, [-1, 0, 1], format='csc')
    for n in range(1, num_time_steps):
        current_time = times[n]
        b = temps[n-1, 1:-1].copy()
        b[0] += r * temp_left(current_time)
        b[-1] += r * temp_right(current_time)
        temps[n, 1:-1] = spsolve(matrix_a, b)
        temps[n, 0] = temp_left(current_time)
        temps[n, -1] = temp_right(current_time)
    fig = plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, num_time_steps-1, 5, dtype=int):
        plt.plot(x_array, temps[idx] - temp_0c, label=f't={times[idx]:.2f} с')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.title('Нестационарное решение без источника (неявная схема)')
    plt.grid(True)
    plt.legend()
    fig.canvas.manager.set_window_title('Task 2b: Transient solution without source (implicit scheme)')
    plt.savefig('task_2b.png')
    plt.show()
    plt.close()
    return x_array, times, temps

# Task 2c: Нестационарный случай с источником (неявная схема)
def task_2c():
    print("Plotting Task 2c: Transient solution with source (implicit scheme)")
    setup_plot()
    temp = np.linspace(temp_left(0), temp_right(0), num_nodes + 1)
    times = np.linspace(0, total_time, num_time_steps)
    temps = np.zeros((num_time_steps, num_nodes + 1))
    temps[0] = temp.copy()
    r = alpha * dt / dx**2
    q_term = heat_source * dt / (density * specific_heat)
    diagonals = [[-r]*(num_nodes-2), [1+2*r]*(num_nodes-1), [-r]*(num_nodes-2)]
    matrix_a = diags(diagonals, [-1, 0, 1], format='csc')
    for n in range(1, num_time_steps):
        current_time = times[n]
        b = temps[n-1, 1:-1].copy() + q_term
        b[0] += r * temp_left(current_time)
        b[-1] += r * temp_right(current_time)
        temps[n, 1:-1] = spsolve(matrix_a, b)
        temps[n, 0] = temp_left(current_time)
        temps[n, -1] = temp_right(current_time)
    fig = plt.figure(figsize=(10, 5))
    for idx in np.linspace(0, num_time_steps-1, 5, dtype=int):
        plt.plot(x_array, temps[idx] - temp_0c, label=f't={times[idx]:.2f} с')
    plt.xlabel('x, м')
    plt.ylabel('T, °C')
    plt.title('Нестационарное решение с источником (неявная схема)')
    plt.grid(True)
    plt.legend()
    fig.canvas.manager.set_window_title('Task 2c: Transient solution with source (implicit scheme)')
    plt.savefig('task_2c.png')
    plt.show()
    plt.close()
    return x_array, times, temps

if __name__ == "__main__":
    setup_plot()
    task_1a()
    task_1b()
    task_1c()
    task_2a()
    task_2b()
    task_2c()