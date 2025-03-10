% Стартовая программа для демонстрации случайных блужданий
n = 500;  % Число частиц
dh = .02;  % Параметр случайного распределения

% Задание вектор-столбцов координат точек
y = 1:n; 
y = zeros(size(y));
y = y'; 
x = zeros(size(y));

% h = plot(x, y, 'k.');  % Вывод начального положения точек
h = line([0], [0]);

axis([-2 2 -2 2]);  % Задание осей

% Определение режима перерисовки и размера точек
set(h, 'EraseMode', 'background', 'MarkerSize', 3, 'LineStyle', '.');
pause(1);  % Пауза для вывода графика на экран

i = 0;  % Начальное значение

dn = 0:0.02:2;
a = 0;
b = 0;
ac = dh * dh / 3;
lh = line([0], [0]);
le = line([0], [0]);
set(lh, 'Color', 'r');
nc = 0;

while nc < 10000  % Не бесконечный цикл
    nc = nc + 1;
    i = i + 1;

    x = x + dh * (2 * rand(n, 1) - 1);  % Случайные смещения x-координаты
    y = y + dh * (2 * rand(n, 1) - 1);  % Случайные смещения y-координаты

    % Смена координат точек на рисунке
    set(h, 'XData', x, 'YData', y, 'Color', 'k');

    r = sqrt(x .* x + y .* y);
    [a, b] = hist(r, dn);
    [a, b] = stairs(a, b);

    % c = exp(-b.*b/2/nc/dh/dh)/sqrt(2*nc*pi*dh*dh)/10;
    c = (b .* 2 * pi * 0.02 * n / 5.7) .* exp(-b .* b / (2 * i * ac)) / (i * ac) / n * 5;

    set(lh, 'XData', b, 'YData', a / n * 5 - 2);
    set(le, 'XData', b, 'YData', c - 2);

    pause(0.01);
end;
