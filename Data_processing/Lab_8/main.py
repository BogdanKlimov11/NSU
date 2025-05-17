import numpy as np
from scipy.linalg import solve
import matplotlib.pyplot as plt
from scipy import integrate

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Task 1: Метод квадратур
def task_1():
    print("Plotting Task 1: Quadrature method")
    setup_plot()
    lower_bound, upper_bound = 0, 1
    step_size = 1/30
    lambda_coeff = 1/2

    def kernel(x, s):
        return x * s

    def f(x):
        return (5/6) * x

    def exact_solution(x):
        return x

    def solve_integral_equation():
        x = np.arange(lower_bound, upper_bound + step_size, step_size)
        n = len(x)
        x_col = x.reshape(-1, 1)
        a = np.zeros((n, n))
        for i in range(n):
            xi = x_col[i, 0]
            a[i, 0] = -step_size * 0.5 * kernel(xi, x[0]) * lambda_coeff
            for j in range(1, n-1):
                a[i, j] = -step_size * 1 * kernel(xi, x[j]) * lambda_coeff
            a[i, n-1] = -step_size * 0.5 * kernel(xi, x[-1]) * lambda_coeff
            a[i, i] += 1
        b_vec = f(x_col).flatten()
        y_approx = solve(a, b_vec)
        return x, y_approx

    x, y_approx = solve_integral_equation()
    fig = plt.figure(figsize=(10, 6))
    plt.plot(x, exact_solution(x), 'r-', label='Точное решение')
    plt.plot(x, y_approx, 'b--', label='Приближенное решение')
    plt.xlabel('x')
    plt.ylabel('y(x)')
    plt.title('Решение интегрального уравнения методом квадратур')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 1: Quadrature method')
    plt.savefig('task_1.png')
    plt.show()
    plt.close()
    print(f"Максимальная ошибка: {np.max(np.abs(y_approx - exact_solution(x))):.3e}")

# Task 2: Аппроксимация вырожденным ядром
def task_2():
    print("Plotting Task 2: Degenerate kernel approximation")
    setup_plot()
    lower_bound, upper_bound = 0, 1
    step_size = 1/20
    lambda_coeff = -1

    x = np.arange(lower_bound, upper_bound, step_size).reshape(-1, 1)
    n = len(x)

    def basis_alpha(x):
        return [x**2, x**3, x**4]

    def basis_beta(s):
        return [s, s**2/2, s**3/6]

    def f(t):
        return np.exp(t) - t

    def solve_equation():
        m = len(basis_alpha(0))
        m_matrix = np.zeros((m, m))
        r = np.zeros((m, 1))
        for i in range(m):
            r[i] = integrate.quad(lambda t: basis_beta(t)[i] * f(t), lower_bound, upper_bound)[0]
            for j in range(m):
                m_matrix[i, j] = -lambda_coeff * integrate.quad(
                    lambda t: basis_beta(t)[i] * basis_alpha(t)[j], lower_bound, upper_bound
                )[0]
        m_matrix += np.eye(m)
        c = solve(m_matrix, r)
        y_approx = lambda_coeff * (c[0] * basis_alpha(x)[0] + c[1] * basis_alpha(x)[1] + c[2] * basis_alpha(x)[2]) + f(x)
        return y_approx

    y_approx = solve_equation()
    y_exact = np.ones_like(x)
    fig = plt.figure(figsize=(10, 6))
    plt.plot(x, y_exact, 'g-', label='Точное решение')
    plt.plot(x, y_approx, 'ro', label='Приближенное решение')
    plt.xlabel('x')
    plt.ylabel('y(x)')
    plt.title('Решение с аппроксимацией вырожденным ядром')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 2: Degenerate kernel approximation')
    plt.savefig('task_2.png')
    plt.show()
    plt.close()

# Task 3: Метод Галеркина-Петрова
def task_3():
    print("Plotting Task 3: Galerkin-Petrov method")
    setup_plot()
    lower_bound, upper_bound = -1, 1
    x = np.linspace(lower_bound, upper_bound, 100).reshape(-1, 1)

    def exact_solution(x):
        return 1 + 6 * x**2

    def basis_phi(x):
        return [x, x**2]

    def basis_psi(x):
        return [1, x]

    def kernel(x, s):
        return x**2 + x * s

    def f(x):
        return 1

    def solve_galerkin():
        a = np.zeros((2, 2))
        b_vec = np.zeros(2)
        for i in range(2):
            b_vec[i] = integrate.dblquad(
                lambda x, s: basis_psi(x)[i] * kernel(x, s) * f(s),
                lower_bound, upper_bound, lower_bound, upper_bound
            )[0]
            for j in range(2):
                term1 = integrate.quad(
                    lambda x: basis_psi(x)[i] * basis_phi(x)[j], lower_bound, upper_bound
                )[0]
                term2 = integrate.dblquad(
                    lambda x, s: basis_psi(x)[i] * kernel(x, s) * basis_phi(s)[j],
                    lower_bound, upper_bound, lower_bound, upper_bound
                )[0]
                a[i, j] = term1 - term2
        c = solve(a, b_vec)
        return lambda x: 1 + c[0] * basis_phi(x)[0] + c[1] * basis_phi(x)[1]

    y_approx = solve_galerkin()
    fig = plt.figure(figsize=(10, 6))
    plt.plot(x, exact_solution(x), 'g-', label='Точное решение')
    plt.plot(x, y_approx(x), 'r--', label='Приближенное решение')
    plt.xlabel('x')
    plt.ylabel('y(x)')
    plt.title('Решение методом Галеркина-Петрова')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 3: Galerkin-Petrov method')
    plt.savefig('task_3.png')
    plt.show()
    plt.close()
    print("Коэффициенты приближенного решения:", solve_galerkin.__closure__[1].cell_contents)

if __name__ == "__main__":
    task_1()
    task_2()
    task_3()