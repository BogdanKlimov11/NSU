clear;

set(0, 'units', 'pixels');
res = get(0, 'screensize');
res_x = res(3);
res_y = res(4);
set(gcf, 'position', [res_x / 2 + 2, (res_y - res_y * 0.8) / 2, res_y * 0.8, res_y * 0.8]);

% Инициализация параметров
n = 300;  % Количество шаров
dt = 0.01;  % Шаг по времени
tmot = 0;  % Текущее время
lx = 200; ly = 200;  % Размеры области
rad(1:n) = 1;  % Радиусы шаров
m(1:n) = 1;  % Массы шаров
g1 = 0.001;  % Ускорение свободного падения
v0 = 100;  % Начальные скорости и координаты из set_random.m
out = set_random(v0);  % Генерация случайных координат частиц
x = out(1,:);  % Координаты по x
y = out(2,:);  % Координаты по y
vx = out(3,:);  % Скорости по x
vy = out(4,:);  % Скорости по y

% Вычисление минимальных и максимальных значений
vx_min = min(vx); 
vy_min = min(vy);
vx_max = max(vx);
vy_max = max(vy);
v_min = sqrt(vx_min^2 + vy_min^2);
v_max = sqrt(vx_max^2 + vy_max^2);

% Настройка графика
h = animatedline('Color', 'k', 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 5 * rad(1));
axis([-lx, lx, -ly, ly]);
axis('square');
grid on;

% Основной цикл
while ishghandle(h)
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);  % Обновление позиций и скоростей
    clearpoints(h);  % Очистка предыдущих точек
    addpoints(h, x, y);  % Добавление новых точек
    pause(0.000001);  % Замедление анимации
end;
