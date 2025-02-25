% Солнце, Земля, Луна
clear;
clear global;
cla;
warning off;

global rad lx ly m n;

n = 10;         % Количество шаров
dt = 0.05;      % Шаг по времени
tmax = 1000.0;  % Максимальное время счета
tmot = 0;       % Текущее время
lx = 200; ly = 200;  % Размеры бильярда
rad(1:n) = 5;   % Радиусы шаров
m(1:n) = 1;     % Массы шаров
g1 = 0.001;     % Ускорение свободного падения
dv = 0.2;

% Начальные скорости и координаты из set_random.m
out = set_random (20);  % Задаем случайные координаты частиц
x = out(1,:);           % Скорости всех направлены по х и равны аргументу
y = out(2,:);
vx = out(3,:);
vy = out(4,:);

% Вычисление минимального и максимального значения
vxmin = min (vx);
vymin = min (vy);
vxmax = max (vx);
vymax = max (vy);
vmin = sqrt (vxmin^2 + vymin^2);
vmax = sqrt (vxmax^2 + vymax^2);

% Вывод на график
h = animatedline (x, y);  % Рисуем шары
set (h, 'LineStyle', 'none', 'color', 'b', 'Marker', 'o', 'MarkerSize', 2.4 * rad(1));
axis ([0, lx, 0, ly]);
axis ('square');

while (tmot < tmax) && ishghandle (h)
    %tmot = tmot + dt;  % Обновление текущего времени (закомментировано)
    %xin = x; yin = y; vxin = vx; vyin = vy;  % Сохранение предыдущих значений (закомментировано)
    %tic
    %Balls_mex (n, dt, g1, x, y, vx, vy, m, rad, [0 0 lx ly]);  % Откомпилированный модуль на С для расчета новых координат и скоростей
    [x, y, vx, vy] = BallsF8 (n, x, y, vx, vy, dt);  % Откомпилированная функция MATLAB, можно посмотреть текст программы
    clearpoints (h);
    addpoints (h, x, y);

    pause (0.0001);
end;
