% Стартовая программа для демонстрации случайных блужданий
n = 500;  % Число частиц
dh = .02;  % Параметр случайного распределения

% Задание вектор-столбцов координат точек
y = 1 : n;
y = y'; x = zeros (size (y));
h = plot (x, y, 'k.');  % Вывод начального положения точек
axis ([-2 2 0 n + 1]);  % Задание осей

% Определение режима перерисовки и размера точек
set (h, 'EraseMode', 'background', 'MarkerSize', 3);
pause (1)  % Пауза для вывода графика на экран

i = 0;  % Начальное значение
cn = 0;

while cn < 1000  % Небесконечный цикл
    cn = cn + 1;
    i = i + 1;
    x = x + dh * (2 * rand (n, 1) - 1);  % Случайные смещения x-координаты

    % Смена координат точек на рисунке
    set (h, 'XData', x, 'YData', y, 'Color', 'k')
    pause (0.01);
end
