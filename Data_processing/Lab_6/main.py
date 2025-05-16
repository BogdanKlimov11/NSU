import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import medfilt
from scipy.linalg import lstsq

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Скользящее среднее
def moving_average(signal, window_size):
    averaged = np.zeros_like(signal)
    for i in range(len(signal)):
        start = max(0, i - window_size)
        end = min(len(signal), i + window_size + 1)
        averaged[i] = np.mean(signal[start:end])
    return averaged

# Гауссово сглаживание
def gaussian_smoothing(signal, window_size, omega=5):
    indices = np.arange(-window_size, window_size + 1)
    gaussian_kernel = np.exp(-4 * np.log(2) * indices ** 2 / omega ** 2)
    gaussian_kernel /= np.sum(gaussian_kernel)
    smoothed = np.zeros_like(signal)
    for i in range(len(signal)):
        start = max(0, i - window_size)
        end = min(len(signal), i + window_size + 1)
        kernel_start = max(0, window_size - i)
        kernel_end = min(2 * window_size + 1, window_size + (len(signal) - i))
        smoothed[i] = np.sum(signal[start:end] * gaussian_kernel[kernel_start:kernel_end])
    return smoothed

# Вычисление спектра (положительные частоты)
def compute_spectrum(signal, n_points, dt):
    yf = fft(signal)
    xf = fftfreq(n_points, dt)
    return xf[:n_points // 2], 2 / n_points * np.abs(yf[:n_points // 2])

# Задача 1: Скользящее среднее для синусоидального сигнала
def task_1():
    print("Plotting Task 1: Moving average smoothing")
    setup_plot()
    n_points = 1000
    time_array = np.linspace(0, 1, n_points)
    dt = time_array[1] - time_array[0]
    signal_freq = 5
    clean_signal = np.sin(2 * np.pi * signal_freq * time_array)
    noise_level = 0.5
    noisy_signal = clean_signal + np.random.normal(0, noise_level, n_points)
    window_size = 5
    averaged_signal = moving_average(noisy_signal, window_size)

    freq_noisy, spectrum_noisy = compute_spectrum(noisy_signal, n_points, dt)
    freq_avg, spectrum_avg = compute_spectrum(averaged_signal, n_points, dt)

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(time_array, noisy_signal, label='Зашумленный сигнал', alpha=0.7)
    ax.plot(time_array, averaged_signal, label='Усредненный сигнал', color='red')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Сигналы во временной области')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(freq_noisy, spectrum_noisy, label='Зашумленный сигнал', alpha=0.7)
    ax.plot(freq_avg, spectrum_avg, label='Усредненный сигнал', color='red')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектры сигналов')
    ax.set_xlim(0, 50)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    zoom_start, zoom_end = 100, 200
    ax.plot(time_array[zoom_start:zoom_end], noisy_signal[zoom_start:zoom_end], label='Зашумленный сигнал', alpha=0.7)
    ax.plot(time_array[zoom_start:zoom_end], averaged_signal[zoom_start:zoom_end], label='Усредненный сигнал', color='red')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Увеличенный участок сигналов')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 1: Moving average smoothing')
    plt.savefig('task_1.png')
    plt.show()
    plt.close()

# Задача 2: Скользящее среднее и гауссово сглаживание
def task_2():
    print("Plotting Task 2: Moving average and Gaussian smoothing")
    setup_plot()
    n_points = 1000
    time_array = np.linspace(0, 1, n_points)
    dt = time_array[1] - time_array[0]
    signal_freq1 = 5
    signal_freq2 = 20
    clean_signal = 0.5 * np.sin(2 * np.pi * signal_freq1 * time_array) + 0.2 * np.sin(2 * np.pi * signal_freq2 * time_array)
    noise_level = 0.3
    noisy_signal = clean_signal + np.random.normal(0, noise_level, n_points)
    window_size = 15
    omega = 10
    ma_signal = moving_average(noisy_signal, window_size)
    gs_signal = gaussian_smoothing(noisy_signal, window_size, omega)

    freq_noisy, spectrum_noisy = compute_spectrum(noisy_signal, n_points, dt)
    freq_ma, spectrum_ma = compute_spectrum(ma_signal, n_points, dt)
    freq_gs, spectrum_gs = compute_spectrum(gs_signal, n_points, dt)

    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(time_array, noisy_signal, label='Зашумленный сигнал', alpha=0.5)
    ax.plot(time_array, ma_signal, label='Скользящее среднее', linewidth=2)
    ax.plot(time_array, gs_signal, label='Гауссово сглаживание', linewidth=2)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Сравнение методов сглаживания')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(freq_noisy, spectrum_noisy, label='Зашумленный сигнал', alpha=0.5)
    ax.plot(freq_ma, spectrum_ma, label='Скользящее среднее')
    ax.plot(freq_gs, spectrum_gs, label='Гауссово сглаживание')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектры сигналов')
    ax.set_xlim(0, 50)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    zoom_start, zoom_end = 100, 300
    ax.plot(time_array[zoom_start:zoom_end], noisy_signal[zoom_start:zoom_end], label='Зашумленный сигнал', alpha=0.5)
    ax.plot(time_array[zoom_start:zoom_end], ma_signal[zoom_start:zoom_end], label='Скользящее среднее')
    ax.plot(time_array[zoom_start:zoom_end], gs_signal[zoom_start:zoom_end], label='Гауссово сглаживание')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Увеличенный участок сигналов')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 2: Moving average and Gaussian smoothing')
    plt.savefig('task_2.png')
    plt.show()
    plt.close()

# Задача 3: Гауссово сглаживание для сигнала с пиками
def task_3():
    print("Plotting Task 3: Gaussian smoothing for peaked signal")
    setup_plot()
    n_points = 1000
    time_array = np.linspace(0, 1, n_points)
    dt = time_array[1] - time_array[0]
    np.random.seed(42)
    peaks_positions = np.random.randint(50, 950, 10)
    signal = np.zeros(n_points)
    signal[peaks_positions] = 1
    noise_level = 0.1
    noisy_signal = signal + np.random.normal(0, noise_level, n_points)
    window_size = 20
    omega_narrow = 5
    omega_wide = 15
    gs_narrow = gaussian_smoothing(noisy_signal, window_size, omega_narrow)
    gs_wide = gaussian_smoothing(noisy_signal, window_size, omega_wide)

    freq_noisy, spectrum_noisy = compute_spectrum(noisy_signal, n_points, dt)
    freq_narrow, spectrum_narrow = compute_spectrum(gs_narrow, n_points, dt)
    freq_wide, spectrum_wide = compute_spectrum(gs_wide, n_points, dt)

    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(time_array, noisy_signal, label='Зашумленный сигнал с пиками', alpha=0.5)
    ax.plot(time_array, gs_narrow, label=f'Гауссово (ω={omega_narrow})', linewidth=2)
    ax.plot(time_array, gs_wide, label=f'Гауссово (ω={omega_wide})', linewidth=2)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Обработка сигнала с пиками')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(freq_noisy, spectrum_noisy, label='Зашумленный сигнал', alpha=0.5)
    ax.plot(freq_narrow, spectrum_narrow, label=f'Гауссово (ω={omega_narrow})')
    ax.plot(freq_wide, spectrum_wide, label=f'Гауссово (ω={omega_wide})')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектры сигналов')
    ax.set_xlim(0, 100)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    zoom_start, zoom_end = 300, 700
    ax.plot(time_array[zoom_start:zoom_end], noisy_signal[zoom_start:zoom_end], label='Зашумленный сигнал', alpha=0.5)
    ax.plot(time_array[zoom_start:zoom_end], gs_narrow[zoom_start:zoom_end], label=f'Гауссово (ω={omega_narrow})')
    ax.plot(time_array[zoom_start:zoom_end], gs_wide[zoom_start:zoom_end], label=f'Гауссово (ω={omega_wide})')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Увеличенный участок с пиками')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 4)
    indices = np.arange(-window_size, window_size + 1)
    kernel_narrow = np.exp(-4 * np.log(2) * indices ** 2 / omega_narrow ** 2)
    kernel_narrow /= np.sum(kernel_narrow)
    kernel_wide = np.exp(-4 * np.log(2) * indices ** 2 / omega_wide ** 2)
    kernel_wide /= np.sum(kernel_wide)
    ax.plot(indices, kernel_narrow, label=f'Узкое ядро (ω={omega_narrow})')
    ax.plot(indices, kernel_wide, label=f'Широкое ядро (ω={omega_wide})')
    ax.set_xlabel('Отсчёты')
    ax.set_ylabel('Вес')
    ax.set_title('Гауссовы ядра (нормированные)')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 3: Gaussian smoothing for peaked signal')
    plt.savefig('task_3.png')
    plt.show()
    plt.close()

# Задача 4: Пороговая и медианная фильтрация
def task_4():
    print("Plotting Task 4: Threshold and median filtering")
    setup_plot()
    def threshold_filter(signal, threshold):
        filtered = signal.copy()
        for i in range(len(signal)):
            if abs(signal[i]) > threshold:
                left = signal[i - 1] if i > 0 else signal[i + 1]
                right = signal[i + 1] if i < len(signal) - 1 else signal[i - 1]
                filtered[i] = (left + right) / 2
        return filtered

    n_points = 1000
    time_array = np.linspace(0, 1, n_points)
    dt = time_array[1] - time_array[0]
    signal_freq1 = 2
    signal_freq2 = 15
    clean_signal = 0.7 * np.sin(2 * np.pi * signal_freq1 * time_array) + 0.3 * np.sin(2 * np.pi * signal_freq2 * time_array)
    np.random.seed(42)
    noisy_signal = clean_signal.copy()
    num_spikes = 25
    spike_indices = np.random.choice(n_points, size=num_spikes, replace=False)
    spike_amplitudes = np.random.uniform(1.5, 10, size=num_spikes) * np.random.choice([-1, 1], size=num_spikes)
    noisy_signal[spike_indices] += spike_amplitudes
    threshold = np.mean(np.abs(noisy_signal)) + 3 * np.std(noisy_signal)
    thresholded = threshold_filter(noisy_signal, threshold)
    filtered_signal = medfilt(thresholded, kernel_size=5)

    freq_clean, spectrum_clean = compute_spectrum(clean_signal, n_points, dt)
    freq_noisy, spectrum_noisy = compute_spectrum(noisy_signal, n_points, dt)
    freq_filtered, spectrum_filtered = compute_spectrum(filtered_signal, n_points, dt)

    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(time_array, clean_signal, 'b', label='Чистый сигнал', alpha=0.7)
    ax.plot(time_array, noisy_signal, 'r', label='Зашумленный сигнал', alpha=0.5)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Сигналы во временной области')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(time_array, clean_signal, 'b', label='Чистый сигнал', alpha=0.7)
    ax.plot(time_array, filtered_signal, 'g', label='Фильтрованный сигнал', alpha=0.8)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Результат фильтрации')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(freq_clean, spectrum_clean, 'b', label='Чистый сигнал')
    ax.plot(freq_noisy, spectrum_noisy, 'r', label='Зашумленный сигнал', alpha=0.5)
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектры сигналов')
    ax.set_xlim(0, 50)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(freq_clean, spectrum_clean, 'b', label='Чистый сигнал')
    ax.plot(freq_filtered, spectrum_filtered, 'g', label='Фильтрованный сигнал')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектр после фильтрации')
    ax.set_xlim(0, 50)
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 4: Threshold and median filtering')
    plt.savefig('task_4.png')
    plt.show()
    plt.close()

# Задача 5: Удаление нелинейного тренда
def task_5():
    print("Plotting Task 5: Nonlinear trend removal")
    setup_plot()
    np.random.seed(42)
    n_points = 1000
    time_array = np.linspace(0, 10, n_points)
    dt = time_array[1] - time_array[0]
    signal_freq1 = 1.5
    signal_freq2 = 5.0
    clean_signal = 0.5 * np.sin(2 * np.pi * signal_freq1 * time_array) + 0.3 * np.sin(2 * np.pi * signal_freq2 * time_array)
    trend = 0.1 * time_array ** 2 - 0.5 * time_array + 0.05 * time_array ** 3
    noise = np.random.normal(0, 0.2, n_points)
    signal_with_trend = clean_signal + trend + noise

    def calculate_bic(y_true, y_pred, k):
        n = len(y_true)
        residuals = y_true - y_pred
        sigma_sq = np.sum(residuals ** 2) / n
        return n * np.log(sigma_sq) + k * np.log(n)

    def create_poly_matrix(t, degree):
        return np.column_stack([t ** i for i in range(degree + 1)])

    max_poly_degree = 6
    bic_values = []
    poly_fits = []

    for degree in range(1, max_poly_degree + 1):
        X = create_poly_matrix(time_array, degree)
        coefficients, _, _, _ = lstsq(X, signal_with_trend)
        y_pred = X @ coefficients
        bic = calculate_bic(signal_with_trend, y_pred, degree + 1)
        bic_values.append(bic)
        poly_fits.append(y_pred)

    optimal_degree = np.argmin(bic_values) + 1
    print(f"Оптимальная степень полинома: {optimal_degree} (минимальный BIC)")

    X_optimal = create_poly_matrix(time_array, optimal_degree)
    coefficients_optimal, _, _, _ = lstsq(X_optimal, signal_with_trend)
    trend_estimate = X_optimal @ coefficients_optimal
    detrended_signal = signal_with_trend - trend_estimate

    freq_original, spectrum_original = compute_spectrum(signal_with_trend, n_points, dt)
    freq_detrended, spectrum_detrended = compute_spectrum(detrended_signal, n_points, dt)
    freq_clean, spectrum_clean = compute_spectrum(clean_signal, n_points, dt)

    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(time_array, signal_with_trend, label='Исходный сигнал с трендом')
    ax.plot(time_array, trend_estimate, label=f'Оценка тренда ({optimal_degree} степени)', color='red')
    ax.plot(time_array, detrended_signal, label='Сигнал без тренда', color='green')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Удаление нелинейного тренда')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(freq_original, spectrum_original, label='Исходный сигнал')
    ax.plot(freq_detrended, spectrum_detrended, label='Без тренда', color='green')
    ax.plot(freq_clean, spectrum_clean, label='Чистый сигнал', linestyle='--')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Сравнение спектров')
    ax.set_xlim(0, 10)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(range(1, max_poly_degree + 1), bic_values, marker='o')
    ax.set_xlabel('Степень полинома')
    ax.set_ylabel('Значение BIC')
    ax.set_title('Критерий BIC для разных степеней полинома')
    ax.set_xticks(range(1, max_poly_degree + 1))
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 4)
    zoom_start, zoom_end = 400, 600
    ax.plot(time_array[zoom_start:zoom_end], signal_with_trend[zoom_start:zoom_end], label='С трендом')
    ax.plot(time_array[zoom_start:zoom_end], detrended_signal[zoom_start:zoom_end], label='Без тренда', color='green')
    ax.plot(time_array[zoom_start:zoom_end], clean_signal[zoom_start:zoom_end], label='Чистый сигнал', linestyle='--')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Увеличенный участок сигналов')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 5: Nonlinear trend removal')
    plt.savefig('task_5.png')
    plt.show()
    plt.close()

    # Дополнительный график: аппроксимация для всех степеней полинома
    fig = plt.figure(figsize=(15, 10))
    for degree in range(1, max_poly_degree + 1):
        ax = fig.add_subplot(2, 3, degree)
        ax.plot(time_array, signal_with_trend, label='Исходный сигнал', alpha=0.7)
        ax.plot(time_array, poly_fits[degree - 1], label=f'Полином {degree} степени', color='red')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Степень {degree}, BIC={bic_values[degree - 1]:.2f}')
        ax.legend()
        ax.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 5: Polynomial fits for different degrees')
    plt.savefig('task_5_poly_fits.png')
    plt.show()
    plt.close()

# Выполнение всех задач
if __name__ == "__main__":
    task_1()
    task_2()
    task_3()
    task_4()
    task_5()