import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Класс для разложения сигнала в ряд Фурье
class FourierSeries:
    def __init__(self, period, n_terms):
        self.period = period
        self.n_terms = n_terms
        self.angular_freq = 2 * np.pi / period

    def rectangular_pulse(self, t, pulse_width, amplitude):
        return amplitude * (np.abs(t % self.period) < pulse_width / 2)

    def sawtooth_pulse(self, t, amplitude):
        phase = (t % self.period) / self.period
        return amplitude * (2 * phase - 1)

    def triangular_pulse(self, t, amplitude):
        phase = (t % self.period) / self.period
        return amplitude * (2 * np.abs(2 * phase - 1) - 1)

    def sine_wave(self, t, amplitude):
        return amplitude * np.sin(2 * np.pi * t / self.period)

    def cosine_wave(self, t, amplitude):
        return amplitude * np.cos(2 * np.pi * t / self.period)

    def get_coefficients(self, func, pulse_width, amplitude):
        t = np.linspace(0, self.period, 1000)
        a0 = (2 / self.period) * np.trapezoid(func(t, pulse_width, amplitude), t)
        an = []
        bn = []
        for n in range(1, self.n_terms + 1):
            an_n = (2 / self.period) * np.trapezoid(func(t, pulse_width, amplitude) * np.cos(n * self.angular_freq * t), t)
            bn_n = (2 / self.period) * np.trapezoid(func(t, pulse_width, amplitude) * np.sin(n * self.angular_freq * t), t)
            an.append(an_n)
            bn.append(bn_n)
        return a0, np.array(an), np.array(bn)

    def approximate(self, t, a0, an, bn):
        result = a0 / 2 * np.ones_like(t)
        for n in range(1, self.n_terms + 1):
            result += an[n-1] * np.cos(n * self.angular_freq * t) + bn[n-1] * np.sin(n * self.angular_freq * t)
        return result

    def predict_spectrum(self, an, bn):
        """Предсказанный спектр на основе коэффициентов Фурье"""
        n = np.arange(1, len(an) + 1)
        magnitude = np.sqrt(an**2 + bn**2)
        return n * self.angular_freq / (2 * np.pi), magnitude

# Задача 1а: Вывод аппроксимированной функции x(t)*
def task_1a():
    print("Plotting Task 1a: Approximated function x(t)*")
    setup_plot()
    period = 1.0
    pulse_width = 0.5
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.rectangular_pulse(t, pulse_width, amplitude)
    a0, an, bn = fs.get_coefficients(fs.rectangular_pulse, pulse_width, amplitude)
    x_t_approx = fs.approximate(t, a0, an, bn)
    print(f"Approximated function coefficients: a0={a0:.4f}, an={an[:5]}, bn={bn[:5]}...")
    fig = plt.figure(figsize=(12, 4))
    plt.plot(t, x_t, 'b-', label='x(t)')
    plt.plot(t, x_t_approx, 'r--', label=f'x*(t), N={n_terms}')
    plt.xlabel('Время, с')
    plt.ylabel('Амплитуда')
    plt.title('Исходный сигнал и аппроксимация')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 1a: Approximated function x(t)*')
    plt.savefig('task_1a.png')
    plt.show()
    plt.close()

# Задача 1б: График погрешности приближения
def task_1b():
    print("Plotting Task 1b: Approximation error")
    setup_plot()
    period = 1.0
    pulse_width = 0.5
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.rectangular_pulse(t, pulse_width, amplitude)
    a0, an, bn = fs.get_coefficients(fs.rectangular_pulse, pulse_width, amplitude)
    x_t_approx = fs.approximate(t, a0, an, bn)
    error = x_t - x_t_approx
    fig = plt.figure(figsize=(12, 4))
    plt.plot(t, error, 'g-', label='Погрешность')
    plt.xlabel('Время, с')
    plt.ylabel('Амплитуда')
    plt.title('Погрешность приближения')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 1b: Approximation error')
    plt.savefig('task_1b.png')
    plt.show()
    plt.close()

# Задача 2а: Проверка алгоритма на косинусном сигнале
def task_2a():
    print("Plotting Task 2a: Cosine signal spectrum")
    setup_plot()
    period = 1.0 / 100  # Период для f = 100 Гц
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.cosine_wave(t, amplitude)
    fft_result = fft(x_t)
    freq = fftfreq(len(t), t[1] - t[0])
    fft_magnitude = np.abs(fft_result)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], fft_magnitude[:len(freq)//2], 'b-', label='Спектр FFT')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Спектр косинусоидального сигнала (f=100 Гц)')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 2a: Cosine signal spectrum')
    plt.savefig('task_2a.png')
    plt.show()
    plt.close()

# Задача 2б: Добавление спектрального коэффициента an
def task_2b():
    print("Plotting Task 2b: Cosine signal spectrum with an coefficient")
    setup_plot()
    period = 1.0 / 100  # Период для f = 100 Гц
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.cosine_wave(t, amplitude)
    # Используем лямбда-функцию для игнорирования pulse_width
    a0, an, bn = fs.get_coefficients(lambda t, _, a: fs.cosine_wave(t, a), 0, amplitude)
    fft_result = fft(x_t)
    freq = fftfreq(len(t), t[1] - t[0])
    fft_magnitude = np.abs(fft_result)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], fft_magnitude[:len(freq)//2], 'b-', label='Спектр FFT')
    plt.plot(100, an[0], 'ro', label=f'a1={an[0]:.4f} at 100 Гц')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Спектр косинусоидального сигнала с коэффициентом an')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 2b: Cosine signal spectrum with an coefficient')
    plt.savefig('task_2b.png')
    plt.show()
    plt.close()

# Задача 3: Сравнение спектра прямоугольного сигнала (FFT vs предсказанный)
def task_3():
    print("Plotting Task 3: Rectangular signal spectrum comparison")
    setup_plot()
    period = 1.0
    pulse_width = 0.5
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.rectangular_pulse(t, pulse_width, amplitude)
    fft_result = fft(x_t)
    freq = fftfreq(len(t), t[1] - t[0])
    fft_magnitude = np.abs(fft_result)
    a0, an, bn = fs.get_coefficients(fs.rectangular_pulse, pulse_width, amplitude)
    pred_freq, pred_magnitude = fs.predict_spectrum(an, bn)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], fft_magnitude[:len(freq)//2], 'b-', label='Спектр FFT')
    plt.stem(pred_freq, pred_magnitude, linefmt='r-', markerfmt='ro', basefmt='r-', label='Предсказанный спектр')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Сравнение спектров прямоугольного сигнала (FFT vs предсказанный)')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 3: Rectangular signal spectrum comparison')
    plt.savefig('task_3.png')
    plt.show()
    plt.close()

# Задача 4: Спектр прямоугольного сигнала через FFT (дублируется с задачей 3, но оставлен для соответствия)
def task_4():
    print("Plotting Task 4: Rectangular signal spectrum via FFT")
    setup_plot()
    period = 1.0
    pulse_width = 0.5
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.rectangular_pulse(t, pulse_width, amplitude)
    fft_result = fft(x_t)
    freq = fftfreq(len(t), t[1] - t[0])
    fft_magnitude = np.abs(fft_result)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], fft_magnitude[:len(freq)//2], 'b-')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Спектр прямоугольного сигнала (FFT)')
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 4: Rectangular signal spectrum via FFT')
    plt.savefig('task_4.png')
    plt.show()
    plt.close()

# Задача 5: Анализ сигнала с шумом
def task_5():
    print("Plotting Task 5: Signal with noise analysis")
    setup_plot()
    period = 1.0
    pulse_width = 0.5
    amplitude = 1.0
    n_terms = 10
    fs = FourierSeries(period, n_terms)
    t = np.linspace(0, 2 * period, 1000)
    x_t = fs.rectangular_pulse(t, pulse_width, amplitude)
    noise = np.random.normal(0, 0.1, len(t))
    x_t_noisy = x_t + noise
    fft_result = fft(x_t_noisy)
    freq = fftfreq(len(t), t[1] - t[0])
    fft_magnitude = np.abs(fft_result)
    fig = plt.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot(t, x_t_noisy, 'b-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Сигнал с шумом')
    ax1.grid(True)
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(freq[:len(freq)//2], fft_magnitude[:len(freq)//2], 'b-')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр сигнала с шумом')
    ax2.grid(True)
    fig.canvas.manager.set_window_title('Task 5: Signal with noise analysis')
    plt.tight_layout()
    plt.savefig('task_5.png')
    plt.show()
    plt.close()
    print("""
        При добавлении шума к сигналу в спектре появляются дополнительные частотные компоненты.
        Это происходит потому что шум имеет случайную природу и содержит широкий спектр частот.
        Основная форма спектра сохраняется, но появляется "шумовой фон" на всех частотах.
        Амплитуда шумовых компонент зависит от уровня добавленного шума.
    """)

if __name__ == "__main__":
    task_1a()
    task_1b()
    task_2a()
    task_2b()
    task_3()
    task_4()
    task_5()