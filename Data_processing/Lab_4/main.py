import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Создание сигнала с прямоугольными импульсами
def create_rectangular_signal(time_array, amplitude=1.0, pulse_width=0.2, period=1.0):
    return amplitude * (np.mod(time_array, period) < pulse_width).astype(float)

# Самописная свертка
def custom_convolution(signal, kernel):
    signal_length = len(signal)
    kernel_length = len(kernel)
    result_length = signal_length + kernel_length - 1
    result = np.zeros(result_length)
    for n in range(result_length):
        for k in range(kernel_length):
            if n - k >= 0 and n - k < signal_length:
                result[n] += signal[n - k] * kernel[k]
    return result

# Задача 1: Свертка прямоугольного сигнала с гауссовым и линейным ядрами
def task_1():
    print("Plotting Task 1: Convolution of rectangular signal")
    setup_plot()
    time_array = np.linspace(0, 5, 1000)
    amplitude = 1.0
    pulse_width = 0.2
    period = 1.0
    signal = create_rectangular_signal(time_array, amplitude, pulse_width, period)

    kernel_array = np.linspace(-2, 2, 100)
    gaussian_kernel = amplitude * np.exp(-kernel_array ** 2)
    linear_kernel = kernel_array

    convolved_gaussian = np.convolve(signal, gaussian_kernel, mode='same')
    convolved_linear_np = np.convolve(signal, linear_kernel, mode='same')
    convolved_linear_custom = custom_convolution(signal, linear_kernel)
    start_idx = (len(convolved_linear_custom) - len(signal)) // 2
    convolved_linear_custom = convolved_linear_custom[start_idx:start_idx + len(signal)]

    fig = plt.figure(figsize=(14, 10))
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.plot(time_array, signal, label="Прямоугольный сигнал", color='dodgerblue')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.plot(time_array, convolved_gaussian, label="Свертка с гауссовым ядром", color='darkorange')
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Свертка с гауссовым ядром')
    ax2.legend()
    ax2.grid(True)

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(time_array, convolved_linear_np, label="Свертка с линейным ядром (np.convolve)", color='forestgreen')
    ax3.plot(time_array, convolved_linear_custom, label="Свертка с линейным ядром (custom)", color='crimson', linestyle='--')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сравнение свертки с линейным ядром')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 1: Convolution of rectangular signal')
    plt.savefig('task_1.png')
    plt.show()
    plt.close()

# Задача 2: Свертка дискретного сигнала
def task_2():
    print("Plotting Task 2: Convolution of discrete signal")
    setup_plot()
    signal = np.array([0, 0, 1, 1, 1, 0, 0])
    kernel = np.array([0.5, 1, 0.5])
    convolved_signal = custom_convolution(signal, kernel)
    time_array = np.arange(len(signal))
    result_time_array = np.arange(len(convolved_signal))

    fig = plt.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.plot(time_array, signal, label="Исходный сигнал", color='dodgerblue', marker='o')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.plot(np.arange(len(kernel)), kernel, label="Ядро свертки", color='forestgreen', marker='o')
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Ядро свертки')
    ax2.legend()
    ax2.grid(True)

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(result_time_array, convolved_signal, label="Результат свертки", color='crimson', marker='o')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Результат свертки')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 2: Convolution of discrete signal')
    plt.savefig('task_2.png')
    plt.show()
    plt.close()

# Проверка теоремы свертки
def check_convolution_theorem(signal, kernel):
    conv_time = custom_convolution(signal, kernel)
    fft_signal = np.fft.fft(signal, n=len(conv_time))
    fft_kernel = np.fft.fft(kernel, n=len(conv_time))
    fft_result = fft_signal * fft_kernel
    conv_freq = np.fft.ifft(fft_result).real
    freq_array = np.fft.fftfreq(len(conv_time))

    fig = plt.figure(figsize=(12, 10))
    ax1 = fig.add_subplot(4, 1, 1)
    ax1.plot(conv_time, label="Свертка во временной области", color='dodgerblue')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Свертка во временной области')
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(4, 1, 2)
    ax2.plot(freq_array[:len(freq_array)//2], np.abs(np.fft.fft(conv_time)[:len(freq_array)//2]),
             label="Спектр свертки (временная область)", color='darkorange')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр свертки (временная область)')
    ax2.legend()
    ax2.grid(True)

    ax3 = fig.add_subplot(4, 1, 3)
    ax3.plot(conv_freq, label="Свертка через частотную область", color='crimson', linestyle='--')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Свертка через частотную область')
    ax3.legend()
    ax3.grid(True)

    ax4 = fig.add_subplot(4, 1, 4)
    ax4.plot(freq_array[:len(freq_array)//2], np.abs(np.fft.fft(conv_freq)[:len(freq_array)//2]),
             label="Спектр свертки (частотная область)", color='forestgreen')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр свертки (частотная область)')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 3: Convolution theorem verification')
    plt.savefig('task_3.png')
    plt.show()
    plt.close()

    return np.allclose(conv_time, conv_freq, atol=1e-10)

# Задача 3: Проверка теоремы свертки
def task_3():
    print("Plotting Task 3: Convolution theorem verification")
    signal = np.array([0, 0, 1, 1, 1, 0, 0])
    kernel = np.array([0.5, 1, 0.5])
    result = check_convolution_theorem(signal, kernel)
    print("Результаты совпадают: теорема о свертке подтверждена!" if result else "Результаты не совпадают: что-то пошло не так.")

# Создание сигнала с несколькими частотами
def create_multi_freq_signal(time_array, freqs=[50, 120, 200]):
    return sum(np.sin(2 * np.pi * f * time_array) for f in freqs)

# Гауссово ядро для фильтрации
def gaussian_kernel(frequencies, peak_freq, fwhm):
    sigma = fwhm / (2 * np.sqrt(2 * np.log(2)))
    return np.exp(-((frequencies - peak_freq) ** 2) / (2 * sigma ** 2))

# Применение гауссова фильтра
def apply_gaussian_filter(signal, frequencies, peak_freq, fwhm):
    fft_signal = np.fft.fft(signal)
    gaussian = gaussian_kernel(frequencies, peak_freq, fwhm)
    fft_filtered = fft_signal * gaussian
    filtered_signal = np.fft.ifft(fft_filtered).real
    return filtered_signal, gaussian

# Задача 4: Сглаживающий фильтр с ядром Гаусса во временной области
def task_4():
    print("Plotting Task 4: Gaussian smoothing filter in time domain")
    setup_plot()
    fs = 1000
    t = np.linspace(-1, 1, 2 * fs, endpoint=False)
    signal = np.sin(2 * np.pi * 50 * t) + 0.5 * np.sin(2 * np.pi * 200 * t)
    
    noisy_signal = signal + 0.3 * np.random.randn(len(t))

    def compute_gaussian_kernel(t_array, p, w, delta=1):
        s = (w * np.sqrt(4 * np.pi)) / np.sqrt(2 * np.pi - 1)
        return np.exp(-delta * ((t_array - p) / s) ** 2)

    kernel = compute_gaussian_kernel(t, p=0, w=0.1)
    kernel /= np.sum(kernel)

    filtered = np.convolve(noisy_signal, kernel, mode='same')

    fig = plt.figure(figsize=(14, 10))
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.plot(t, noisy_signal, label='Зашумлённый сигнал', color='dodgerblue')
    ax1.set_title('Зашумлённый сигнал')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.grid(True)
    ax1.legend()

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.plot(t, kernel, label='Гауссово ядро', color='darkorange')
    ax2.set_title('Гауссово ядро')
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('Амплитуда')
    ax2.grid(True)
    ax2.legend()

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(t, filtered, label='Сглаженный сигнал', color='crimson')
    ax3.set_title('Сглаживание во временной области')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 4: Gaussian smoothing filter in time domain')
    plt.savefig('task_4.png')
    plt.show()
    plt.close()

# Задача 5: Гауссовы фильтры (узкополосный и низкочастотный)
def task_5():
    print("Plotting Task 5: Gaussian filter application")
    setup_plot()
    fs = 1000
    time_array = np.linspace(0, 1, fs, endpoint=False)
    frequencies = np.fft.fftfreq(len(time_array), d=1/fs)
    signal = create_multi_freq_signal(time_array)
    signal_120hz = np.sin(2 * np.pi * 120 * time_array)

    filtered_signal_narrowband, gaussian_narrowband = apply_gaussian_filter(signal, frequencies, peak_freq=120, fwhm=10)
    filtered_signal_lowpass, gaussian_lowpass = apply_gaussian_filter(signal, frequencies, peak_freq=0, fwhm=100)

    fig = plt.figure(figsize=(14, 10))
    ax1 = fig.add_subplot(3, 2, 1)
    ax1.plot(time_array, signal, color='dodgerblue')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(frequencies[:len(frequencies)//2], np.abs(np.fft.fft(signal)[:len(frequencies)//2]), color='dodgerblue')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр исходного сигнала')
    ax2.grid(True)

    ax3 = fig.add_subplot(3, 2, 3)
    ax3.plot(frequencies[:len(frequencies)//2], gaussian_narrowband[:len(frequencies)//2], color='darkorange')
    ax3.set_xlabel('Частота, Гц')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Гауссово ядро (узкополосный фильтр)')
    ax3.grid(True)

    ax4 = fig.add_subplot(3, 2, 4)
    ax4.plot(time_array, filtered_signal_narrowband, color='darkorange', label="Отфильтрованный сигнал")
    ax4.plot(time_array, signal_120hz, color='gray', linestyle='--', label="Сигнал 120 Гц")
    ax4.set_xlabel('Время, с')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Отфильтрованный сигнал (узкополосный)')
    ax4.legend()
    ax4.grid(True)

    ax5 = fig.add_subplot(3, 2, 5)
    ax5.plot(frequencies[:len(frequencies)//2], gaussian_lowpass[:len(frequencies)//2], color='forestgreen')
    ax5.set_xlabel('Частота, Гц')
    ax5.set_ylabel('Амплитуда')
    ax5.set_title('Гауссово ядро (низкочастотный фильтр)')
    ax5.grid(True)

    ax6 = fig.add_subplot(3, 2, 6)
    ax6.plot(time_array, filtered_signal_lowpass, color='forestgreen', label="Отфильтрованный сигнал")
    ax6.plot(time_array, signal_120hz, color='gray', linestyle='--', label="Сигнал 120 Гц")
    ax6.set_xlabel('Время, с')
    ax6.set_ylabel('Амплитуда')
    ax6.set_title('Отфильтрованный сигнал (низкочастотный)')
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 5: Gaussian filter application')
    plt.savefig('task_5.png')
    plt.show()
    plt.close()

# Окно Планка
def planck_taper(length, epsilon=0.1):
    n = np.arange(length)
    return 1 / (1 + np.exp((2 * epsilon * length / (n - (length - 1) / 2)) - (2 * epsilon * length / (length - 1))))

# Полосовой фильтр с окном Планка
def bandpass_filter(signal, fs, lowcut, highcut, epsilon=0.1):
    fft_signal = np.fft.fft(signal)
    frequencies = np.fft.fftfreq(len(signal), d=1/fs)
    window = np.zeros(len(signal))
    freq_mask = (np.abs(frequencies) >= lowcut) & (np.abs(frequencies) <= highcut)
    window[freq_mask] = 1
    window *= planck_taper(len(signal), epsilon)
    fft_filtered = fft_signal * window
    filtered_signal = np.fft.ifft(fft_filtered).real
    return filtered_signal, window

# Задача 6: Полосовой фильтр с окном Планка
def task_6():
    print("Plotting Task 6: Planck taper bandpass filter")
    setup_plot()
    fs = 1000
    time_array = np.linspace(0, 1, fs, endpoint=False)
    signal = create_multi_freq_signal(time_array)
    signal_120hz = np.sin(2 * np.pi * 120 * time_array)
    lowcut = 115
    highcut = 125
    filtered_signal, window = bandpass_filter(signal, fs, lowcut, highcut, epsilon=0.1)
    frequencies = np.fft.fftfreq(len(time_array), d=1/fs)

    fig = plt.figure(figsize=(14, 10))
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.plot(time_array, signal, color='dodgerblue')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.plot(frequencies[:len(frequencies)//2], window[:len(frequencies)//2], color='darkorange')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Частотная характеристика фильтра (окно Планка)')
    ax2.grid(True)

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.plot(time_array, filtered_signal, color='forestgreen', label="Отфильтрованный сигнал")
    ax3.plot(time_array, signal_120hz, color='gray', linestyle='--', label="Сигнал 120 Гц")
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Отфильтрованный сигнал')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 6: Planck taper bandpass filter')
    plt.savefig('task_6.png')
    plt.show()
    plt.close()

# Задача 7: Сравнение фильтров для зашумленного сигнала
def task_7():
    print("Plotting Task 7: Gaussian vs Planck filter for noisy signal")
    setup_plot()
    fs = 1000
    time_array = np.linspace(0, 1, fs, endpoint=False)
    signal = create_multi_freq_signal(time_array)
    signal_120hz = np.sin(2 * np.pi * 120 * time_array)
    noise = 0.5 * np.random.normal(size=len(time_array))
    noisy_signal = signal + noise
    frequencies = np.fft.fftfreq(len(time_array), d=1/fs)

    filtered_signal_gaussian, gaussian_narrowband = apply_gaussian_filter(noisy_signal, frequencies, peak_freq=120, fwhm=10)
    lowcut = 115
    highcut = 125
    filtered_signal_planck, window = bandpass_filter(noisy_signal, fs, lowcut, highcut, epsilon=0.1)

    fft_noisy = np.fft.fft(noisy_signal)
    fft_gaussian = np.fft.fft(filtered_signal_gaussian)
    fft_planck = np.fft.fft(filtered_signal_planck)

    fig = plt.figure(figsize=(14, 12))
    ax1 = fig.add_subplot(4, 2, 1)
    ax1.plot(time_array, noisy_signal, color='dodgerblue')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Зашумленный сигнал')
    ax1.grid(True)

    ax2 = fig.add_subplot(4, 2, 2)
    ax2.plot(frequencies[:len(frequencies)//2], np.abs(fft_noisy[:len(frequencies)//2]), color='dodgerblue')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр зашумленного сигнала')
    ax2.grid(True)

    ax3 = fig.add_subplot(4, 2, 3)
    ax3.plot(time_array, filtered_signal_gaussian, color='darkorange', label="Отфильтрованный сигнал")
    ax3.plot(time_array, signal_120hz, color='gray', linestyle='--', label="Сигнал 120 Гц")
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Отфильтрованный сигнал (гауссов фильтр)')
    ax3.legend()
    ax3.grid(True)

    ax4 = fig.add_subplot(4, 2, 4)
    ax4.plot(frequencies[:len(frequencies)//2], np.abs(fft_gaussian[:len(frequencies)//2]), color='darkorange')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр (гауссов фильтр)')
    ax4.grid(True)

    ax5 = fig.add_subplot(4, 2, 5)
    ax5.plot(time_array, filtered_signal_planck, color='forestgreen', label="Отфильтрованный сигнал")
    ax5.plot(time_array, signal_120hz, color='gray', linestyle='--', label="Сигнал 120 Гц")
    ax5.set_xlabel('Время, с')
    ax5.set_ylabel('Амплитуда')
    ax5.set_title('Отфильтрованный сигнал (окно Планка)')
    ax5.legend()
    ax5.grid(True)

    ax6 = fig.add_subplot(4, 2, 6)
    ax6.plot(frequencies[:len(frequencies)//2], np.abs(fft_planck[:len(frequencies)//2]), color='forestgreen')
    ax6.set_xlabel('Частота, Гц')
    ax6.set_ylabel('Амплитуда')
    ax6.set_title('Спектр (окно Планка)')
    ax6.grid(True)

    plt.tight_layout()
    plt.subplots_adjust(hspace=0.6)
    fig.canvas.manager.set_window_title('Task 7: Gaussian vs Planck filter for noisy signal')
    plt.savefig('task_7.png')
    plt.show()
    plt.close()

# Выполнение всех задач
if __name__ == "__main__":
    task_1()
    task_2()
    task_3()
    task_4()
    task_5()
    task_6()
    task_7()