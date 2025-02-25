clear;

set(0, 'units', 'pixels');
res = get(0, 'screensize');
res_x = res(3);
res_y = res(4);
set(gcf, 'position', [res_x * 0.1, (res_y - res_y * 0.8) / 2, res_x * 0.9, res_y * 0.8]);

% Инициализация параметров
n = 100;  % Количество шаров
dt = 0.1;  % Шаг по времени
tmot = 0;  % Текущее время
lx = 200; ly = 200;  % Размеры области
rad(1:n) = 1;  % Радиусы шаров
m(1:n) = 1;  % Массы шаров
g1 = 0.001;  % Ускорение свободного падения
v0 = 100;  % Начальные скорости и координаты из set_random.m
out = set_random(v0);  % Генерация случайных координат частиц
x = out(1, :);  % Координаты по x
y = out(2, :);  % Координаты по y
vx = out(3, :);  % Скорости по x
vy = out(4, :);  % Скорости по y

% Вычисление минимальных и максимальных значений
vx_min = min(vx); 
vy_min = min(vy);
vx_max = max(vx);
vy_max = max(vy);
v_min = sqrt(vx_min^2 + vy_min^2);
v_max = sqrt(vx_max^2 + vy_max^2);

bin = 5; 
E = n * v0^2 / 2; 
border = 2 * max(vx);

% Построение графиков
subplot(1, 3, 1);
x_teor = -border:border;
y_teor = bin * sqrt((1 / (2 * pi)) * (n / E)) * exp(-x_teor.^2 * n / (2 * E));
plot(x_teor, y_teor, 'LineWidth', 5); hold on;
h = histogram(vx, -border:bin:border, 'Normalization', 'probability'); hold off;
title('Vx');

subplot(1, 3, 2);
x_teor = 0:2 * border;
y_teor = x_teor * bin * (n / E) .* exp(-x_teor.^2 * n / (2 * E));
plot(x_teor, y_teor, 'LineWidth', 5); hold on;
v = sqrt(vx.^2 + vy.^2);
h2 = histogram(v, 0:bin:2 * border, 'Normalization', 'probability'); hold off;
title('|V|');

subplot(1, 3, 3);
x_teor = 0:border^2;
y_teor = 100 * bin * (n / E) .* exp(-x_teor * n / E);
plot(x_teor, y_teor, 'LineWidth', 5); hold on;
e = (vx.^2 + vy.^2) / 2;
h3 = histogram(e, 0:100 * bin:border^2, 'Normalization', 'probability'); hold off;
title('E');

% Инициализация накопителей
accum = []; 
accum2 = []; 
accum3 = []; 
t = 0;

% Основной цикл
while ishghandle(h)
    t = t + dt;
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);  % Обновление позиций и скоростей
    v = sqrt(vx.^2 + vy.^2);  % Скорости
    e = (vx.^2 + vy.^2) / 2;  % Энергия
    if t > 3  % После 3 секунд включаем накопление данных
        accum = [accum vx];
        accum2 = [accum2 v];
        accum3 = [accum3 e];
        h.Data = accum;
        h2.Data = accum2;
        h3.Data = accum3;
    else
        h.Data = vx;
        h2.Data = v;
        h3.Data = e;
    end;
    pause(0.00001);  % Замедление анимации
end;
