% Стартовая программа для демонстрации случайных блужданий
n = 1000;  % Число частиц
dh = .02;  % Параметр случайного распределения

% Задание вектор-столбцов координат точек
y = (1:n) / 2;
y = y'; 
x = zeros(size(y));

h = plot(x, y, 'k.');  % Вывод начального положения точек
axis([-2 2 0 500 + 1]);  % Задание осей

% Определение режима перерисовки и размера точек
set(h, 'EraseMode', 'background', 'MarkerSize', 3);
pause(1);  % Пауза для вывода графика на экран

i = 0;  % Начальное значение

dn = -2:0.05:2;
a = 0;
b = 0;
ln = line(b, a);
le = line;
lq = line;
set(le, 'Marker', 'x', 'Color', 'r');
nc = 0;
xx = -1:0.05:1;

while nc < 100000  % Не бесконечный цикл
    nc = nc + 1;
    i = i + 1;

    x = x + dh * (2 * rand(n, 1) - 1) + dh * 0.05;  % Случайные смещения x-координаты

    % Смена координат точек на рисунке
    cm = 0;
    for ic = 1:n
        if x(ic) > 1
            x(ic) = 1; 
        end;
        cm = cm + x(ic);
    end;
    cm = cm / n;

    set(h, 'XData', x, 'YData', y, 'Color', 'k');
    [a, b] = hist(x, dn);
    [a, b] = stairs(a, b);

    c = n * exp(-(xx - 1) / (cm - 1));
    set(ln, 'XData', b, 'YData', a);
    set(le, 'XData', cm, 'YData', 250);
    set(lq, 'XData', xx, 'YData', c);

    pause(0.01);
end;
