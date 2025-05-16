import math
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

# Создание временного массива
def create_time_array(start, end, step=0.0001):
    return np.arange(start, end, step)

# Применение низкочастотного фильтра (LPF)
def low_pass_filter(time_array, signal, cutoff_freq):
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    signal_fft = np.fft.fft(signal)
    wc = 2 * math.pi * cutoff_freq
    h = -wc**2 / (-(2 * math.pi * freq_array)**2 + 1j * 2**0.5 * wc * (2 * math.pi * freq_array) + 1)
    return np.fft.ifft(signal_fft * np.abs(h))

# Применение высокочастотного фильтра (HPF)
def high_pass_filter(time_array, signal, cutoff_freq):
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    signal_fft = np.fft.fft(signal)
    wc = 2 * math.pi * cutoff_freq
    h = (2 * math.pi * freq_array)**2 / (-(2 * math.pi * freq_array)**2 + 1j * 2**0.5 * wc * (2 * math.pi * freq_array) + wc**2)
    return np.fft.ifft(signal_fft * np.abs(h))

# Применение полосового фильтра (BPF)
def band_pass_filter(time_array, signal, cutoff_freq_a, cutoff_freq_b):
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    signal_fft = np.fft.fft(signal)
    wc_a = 2 * math.pi * cutoff_freq_a
    wc_b = 2 * math.pi * cutoff_freq_b
    h_lpf = (2 * math.pi * freq_array)**2 / (-(2 * math.pi * freq_array)**2 + 1j * 2**0.5 * wc_b * (2 * math.pi * freq_array) + wc_b**2)
    h_hpf = wc_a**2 / (-wc_a**2 + 1j * 2**0.5 * wc_a * (2 * math.pi * freq_array) + 1)
    return np.fft.ifft(signal_fft * h_lpf * h_hpf * (-1))

# Применение заградительного фильтра (BSF)
def band_stop_filter(time_array, signal, cutoff_freq_a, cutoff_freq_b):
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    signal_fft = np.fft.fft(signal)
    wc_a = 2 * math.pi * cutoff_freq_a
    wc_b = 2 * math.pi * cutoff_freq_b
    h_lpf = (2 * math.pi * freq_array)**2 / (-(2 * math.pi * freq_array)**2 + 1j * 2**0.5 * wc_b * (2 * math.pi * freq_array) + wc_b**2)
    h_hpf = wc_a**2 / (-wc_a**2 + 1j * 2**0.5 * wc_a * (2 * math.pi * freq_array) + 1)
    return np.fft.ifft(signal_fft * (1 + h_lpf * h_hpf))

# Применение фильтра n-го порядка
def nth_order_filter(time_array, signal, cutoff_freq, order):
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    signal_fft = np.fft.fft(signal)
    wc = 2 * math.pi * cutoff_freq
    h = 1 / (1 + (-1)**order * (1j * 2 * math.pi * freq_array / wc)**(2 * order))
    return np.fft.ifft(signal_fft * h)

# Создание сигнала
def create_signal(time_array, frequencies):
    w = [2 * math.pi * f for f in frequencies]
    signal = np.zeros_like(time_array)
    for i in range(len(frequencies)):
        signal += np.cos(w[i] * time_array)
    return signal

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Задача 1: Исходный сигнал и его спектр
def task_1():
    print("Plotting Task 1: Original signal and its spectrum")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    signal_fft = np.fft.fft(original_signal)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    ax1.plot(time_array, original_signal, 'b-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(signal_fft[:len(time_array)//20]), 'r--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр сигнала')
    ax2.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 1: Original signal and its spectrum')
    plt.savefig('task_1.png')
    plt.show()
    plt.close()

# Задача 2: Применение LPF и сравнение спектров
def task_2():
    print("Plotting Task 2: LPF signal and spectrum comparison")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    reference_signal = np.cos(2 * math.pi * 50 * time_array)
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    original_fft = np.fft.fft(original_signal)
    filtered_signal = low_pass_filter(time_array, original_signal, 120)
    filtered_fft = np.fft.fft(filtered_signal)

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
    ax1.plot(time_array, original_signal, 'g-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(original_fft[:len(time_array)//20]), 'm--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр исходного сигнала')
    ax2.grid(True)

    ax3.plot(time_array, filtered_signal.real, 'b-', label='ФНЧ')
    ax3.plot(time_array, reference_signal, 'r--', label='Эталон')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал после ФНЧ')
    ax3.legend()
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(filtered_fft[:len(time_array)//20]), 'c--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр после ФНЧ')
    ax4.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 2: LPF signal and spectrum comparison')
    plt.savefig('task_2.png')
    plt.show()
    plt.close()

# Задача 3: Сравнение сигналов после LPF
def task_3():
    print("Plotting Task 3: LPF signals with different frequencies")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    signal_1 = create_signal(time_array, [50, 150, 450])
    signal_2 = create_signal(time_array, [50, 450])
    reference_signal = np.cos(2 * math.pi * 50 * time_array)
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    filtered_signal_1 = low_pass_filter(time_array, signal_1, 120)
    filtered_signal_2 = low_pass_filter(time_array, signal_2, 120)
    filtered_fft_1 = np.fft.fft(filtered_signal_1)
    filtered_fft_2 = np.fft.fft(filtered_signal_2)

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
    ax1.plot(time_array, filtered_signal_1.real, 'b-', label='ФНЧ')
    ax1.plot(time_array, reference_signal, 'r--', label='Эталон')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Сигнал (50, 150, 450 Гц) после ФНЧ')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(filtered_fft_1[:len(time_array)//20]), 'm--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр (50, 150, 450 Гц) после ФНЧ')
    ax2.grid(True)

    ax3.plot(time_array, filtered_signal_2.real, 'g-', label='ФНЧ')
    ax3.plot(time_array, reference_signal, 'r--', label='Эталон')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал (50, 450 Гц) после ФНЧ')
    ax3.legend()
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(filtered_fft_2[:len(time_array)//20]), 'c--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр (50, 450 Гц) после ФНЧ')
    ax4.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 3: LPF signals with different frequencies')
    plt.savefig('task_3.png')
    plt.show()
    plt.close()

# Задача 4: Применение HPF и сравнение спектров
def task_4():
    print("Plotting Task 4: HPF signal and spectrum comparison")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    reference_signal = np.cos(2 * math.pi * 450 * time_array)
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    original_fft = np.fft.fft(original_signal)
    filtered_signal = high_pass_filter(time_array, original_signal, 300)
    filtered_fft = np.fft.fft(filtered_signal)

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
    ax1.plot(time_array, original_signal, 'g-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(original_fft[:len(time_array)//20]), 'm--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр исходного сигнала')
    ax2.grid(True)

    ax3.plot(time_array, filtered_signal.real, 'b-', label='ФВЧ')
    ax3.plot(time_array, reference_signal, 'r--', label='Эталон')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал после ФВЧ')
    ax3.legend()
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(filtered_fft[:len(time_array)//20]), 'c--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр после ФВЧ')
    ax4.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 4: HPF signal and spectrum comparison')
    plt.savefig('task_4.png')
    plt.show()
    plt.close()

# Задача 5: Применение BPF и BSF
def task_5():
    print("Plotting Task 5: BPF and BSF signals and spectra")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    bpf_signal = band_pass_filter(time_array, original_signal, 100, 200)
    bsf_signal = band_stop_filter(time_array, original_signal, 100, 250)
    bpf_fft = np.fft.fft(bpf_signal)
    bsf_fft = np.fft.fft(bsf_signal)

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
    ax1.plot(time_array, bpf_signal.real, 'b-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Сигнал после ПФ')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(bpf_fft[:len(time_array)//20]), 'r--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр после ПФ')
    ax2.grid(True)

    ax3.plot(time_array, bsf_signal.real, 'g-')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал после ЗФ')
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(bsf_fft[:len(time_array)//20]), 'm--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр после ЗФ')
    ax4.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 5: BPF and BSF signals and spectra')
    plt.savefig('task_5.png')
    plt.show()
    plt.close()

# Задача 6: Сравнение LPF и n-го порядка
def task_6():
    print("Plotting Task 6: LPF vs nth-order filter comparison")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    original_fft = np.fft.fft(original_signal)
    lpf_signal = low_pass_filter(time_array, original_signal, 120)
    nth_signal_5 = nth_order_filter(time_array, original_signal, 120, 5)
    nth_signal_2 = nth_order_filter(time_array, original_signal, 120, 2)
    lpf_fft = np.fft.fft(lpf_signal)
    nth_fft_5 = np.fft.fft(nth_signal_5)
    nth_fft_2 = np.fft.fft(nth_signal_2)

    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(12, 8))
    ax1.plot(time_array, original_signal, 'g-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(original_fft[:len(time_array)//20]), 'r--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр исходного сигнала')
    ax2.grid(True)

    ax3.plot(time_array, lpf_signal.real, 'b-')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал после ФНЧ')
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(lpf_fft[:len(time_array)//20]), 'c--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр после ФНЧ')
    ax4.grid(True)

    ax5.plot(time_array, nth_signal_5.real, 'm-')
    ax5.set_xlabel('Время, с')
    ax5.set_ylabel('Амплитуда')
    ax5.set_title('Сигнал после фильтра 5-го порядка')
    ax5.grid(True)

    ax6.plot(freq_array[:len(time_array)//20], np.abs(nth_fft_5[:len(time_array)//20]), 'k--', label='n=5')
    ax6.plot(freq_array[:len(time_array)//20], np.abs(nth_fft_2[:len(time_array)//20]), 'y--', label='n=2')
    ax6.set_xlabel('Частота, Гц')
    ax6.set_ylabel('Амплитуда')
    ax6.set_title('Спектр после фильтра n-го порядка')
    ax6.legend()
    ax6.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 6: LPF vs nth-order filter comparison')
    plt.savefig('task_6.png')
    plt.show()
    plt.close()

# Задача 8: Сравнение фильтра Баттерворта и n-го порядка
def task_8():
    print("Plotting Task 8: Butterworth vs nth-order filter comparison")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    original_fft = np.fft.fft(original_signal)
    nth_signal = nth_order_filter(time_array, original_signal, 120, 5)
    nth_fft = np.fft.fft(nth_signal)
    b, a = signal.butter(5, 120 * 2 / 10000)
    butter_signal = signal.filtfilt(b, a, original_signal)
    butter_fft = np.fft.fft(butter_signal)

    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(12, 8))
    ax1.plot(time_array, original_signal, 'g-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Исходный сигнал')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(original_fft[:len(time_array)//20]), 'r--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр исходного сигнала')
    ax2.grid(True)

    ax3.plot(time_array, butter_signal, 'b-')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал после фильтра Баттерворта')
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(butter_fft[:len(time_array)//20]), 'c--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр после фильтра Баттерворта')
    ax4.grid(True)

    ax5.plot(time_array, nth_signal.real, 'm-')
    ax5.set_xlabel('Время, с')
    ax5.set_ylabel('Амплитуда')
    ax5.set_title('Сигнал после фильтра 5-го порядка')
    ax5.grid(True)

    ax6.plot(freq_array[:len(time_array)//20], np.abs(nth_fft[:len(time_array)//20]), 'k--')
    ax6.set_xlabel('Частота, Гц')
    ax6.set_ylabel('Амплитуда')
    ax6.set_title('Спектр после фильтра 5-го порядка')
    ax6.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 8: Butterworth vs nth-order filter comparison')
    plt.savefig('task_8.png')
    plt.show()
    plt.close()

# Задача 9: Фильтрация сигнала с шумом
def task_9():
    print("Plotting Task 9: Noise signal filtering")
    setup_plot()
    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450]) + 2 * np.random.random(len(time_array))
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    original_fft = np.fft.fft(original_signal)
    nth_signal = nth_order_filter(time_array, original_signal, 120, 5)
    nth_fft = np.fft.fft(nth_signal)
    b, a = signal.butter(5, 120 * 2 / 10000)
    butter_signal = signal.filtfilt(b, a, original_signal)
    butter_fft = np.fft.fft(butter_signal)

    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(12, 8))
    ax1.plot(time_array, original_signal, 'g-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Сигнал с шумом')
    ax1.grid(True)

    ax2.plot(freq_array[:len(time_array)//20], np.abs(original_fft[:len(time_array)//20]), 'r--')
    ax2.set_xlabel('Частота, Гц')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Спектр сигнала с шумом')
    ax2.grid(True)

    ax3.plot(time_array, butter_signal, 'b-')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Сигнал после фильтра Баттерворта')
    ax3.grid(True)

    ax4.plot(freq_array[:len(time_array)//20], np.abs(butter_fft[:len(time_array)//20]), 'c--')
    ax4.set_xlabel('Частота, Гц')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Спектр после фильтра Баттерворта')
    ax4.grid(True)

    ax5.plot(time_array, nth_signal.real, 'm-')
    ax5.set_xlabel('Время, с')
    ax5.set_ylabel('Амплитуда')
    ax5.set_title('Сигнал после фильтра 5-го порядка')
    ax5.grid(True)

    ax6.plot(freq_array[:len(time_array)//20], np.abs(nth_fft[:len(time_array)//20]), 'k--')
    ax6.set_xlabel('Частота, Гц')
    ax6.set_ylabel('Амплитуда')
    ax6.set_title('Спектр после фильтра 5-го порядка')
    ax6.grid(True)
    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 9: Noise signal filtering')
    plt.savefig('task_9.png')
    plt.show()
    plt.close()

# Задача 10: Сравнение фильтров Чебышева и Баттерворта
def task_10():
    print("Plotting Task 10: Chebyshev vs Butterworth filter comparison")
    setup_plot()

    def chebyshev_polynomial(n, x):
        if n == 0:
            return 1
        elif n == 1:
            return x
        return 2 * x * chebyshev_polynomial(n - 1, x) - chebyshev_polynomial(n - 2, x)

    def chebyshev_filter_response(n, epsilon, wc, w):
        Tn = chebyshev_polynomial(n, w / wc)
        return 1 / (1 + epsilon**2 * Tn**2)

    time_array = create_time_array(-0.1, 0.1)
    original_signal = create_signal(time_array, [50, 150, 450])
    pure_signal = np.cos(2 * math.pi * 50 * time_array)
    freq_array = np.linspace(1, 1 / (time_array[1] - time_array[0]), len(time_array))
    original_fft = np.fft.fft(original_signal)
    wc = 2 * math.pi * 120
    H_squared_cheb = chebyshev_filter_response(5, 0.4, wc, 2 * math.pi * freq_array)
    cheb_signal = np.fft.ifft(original_fft * np.sqrt(H_squared_cheb)).real
    b, a = signal.butter(5, 120 * 2 / 10000)
    butter_signal = signal.filtfilt(b, a, original_signal)
    butter_fft = np.fft.fft(butter_signal)
    cheb_fft = np.fft.fft(cheb_signal)
    pure_fft = np.fft.fft(pure_signal)

    w_butter, h_butter = signal.freqz(b, a, worN=len(freq_array), fs=1/(time_array[1] - time_array[0]))
    freq_butter = w_butter / (2 * np.pi)
    h_butter_mag = np.abs(h_butter)
    h_cheb_mag = np.sqrt(H_squared_cheb)

    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(12, 8))
    ax1.plot(time_array, butter_signal, 'b-')
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Сигнал после фильтра Баттерворта (n=5)')
    ax1.grid(True)

    ax2.plot(time_array, cheb_signal, 'g-')
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Сигнал после фильтра Чебышева (n=5)')
    ax2.grid(True)

    ax3.plot(time_array, pure_signal, 'r-')
    ax3.set_xlabel('Время, с')
    ax3.set_ylabel('Амплитуда')
    ax3.set_title('Чистый сигнал 50 Гц')
    ax3.grid(True)

    ax4.plot(time_array, butter_signal, 'b-', label='Баттерворт')
    ax4.plot(time_array, cheb_signal, 'g-', label='Чебышев')
    ax4.plot(time_array, pure_signal, 'r--', label='50 Гц')
    ax4.set_xlabel('Время, с')
    ax4.set_ylabel('Амплитуда')
    ax4.set_title('Сравнение сигналов')
    ax4.legend()
    ax4.grid(True)

    ax5.plot(freq_array[:len(time_array)//20], np.abs(butter_fft[:len(time_array)//20]), 'b-', label='Баттерворт')
    ax5.plot(freq_array[:len(time_array)//20], np.abs(cheb_fft[:len(time_array)//20]), 'g-', label='Чебышев')
    ax5.plot(freq_array[:len(time_array)//20], np.abs(pure_fft[:len(time_array)//20]), 'r--', label='50 Гц')
    ax5.set_xlabel('Частота, Гц')
    ax5.set_ylabel('Амплитуда')
    ax5.set_title('Сравнение спектров')
    ax5.legend()
    ax5.grid(True)

    ax6.plot(freq_butter[:len(freq_array)//20], h_butter_mag[:len(freq_array)//20], 'b-', label='Баттерворт')
    ax6.plot(freq_array[:len(freq_array)//20], h_cheb_mag[:len(freq_array)//20], 'g-', label='Чебышев')
    ax6.set_xlabel('Частота, Гц')
    ax6.set_ylabel('Коэффициент усиления')
    ax6.set_title('АЧХ фильтров')
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 10: Chebyshev vs Butterworth filter comparison')
    plt.savefig('task_10.png')
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
    task_8()
    task_9()
    task_10()