clear;
clear global;

set(0, 'units', 'pixels');
res = get(0, 'screensize');
resX = res(3);
resY = res(4);
set(gcf, 'position', [resX * 0.1, (resY - resY * 0.8) / 2, resX * 0.9, resY * 0.8]);

global rad lx ly m n;
n = 100;  % Количество частиц
dt = 0.1;  % Шаг по времени
tmot = 0;  % Текущее время
lx = 200; ly = 200;  % Размеры области
rad(1:n) = 1;  % Радиусы частиц
m(1:n) = 1;  % Массы частиц
g1 = 0.001;  % Ускорение гравитации
v0 = 100;  % Начальная скорость
out = set_random(v0);  % Задаём случайные начальные координаты и скорости

x = out(1, :);  % Координаты по оси x
y = out(2, :);  % Координаты по оси y
vx = out(3, :);  % Скорости по оси x
vy = out(4, :);  % Скорости по оси y

vxmin = min(vx);  % Минимальная скорость по оси x
vymin = min(vy);  % Минимальная скорость по оси y
vxmax = max(vx);  % Максимальная скорость по оси x
vymax = max(vy);  % Максимальная скорость по оси y
vmin = sqrt(vxmin^2 + vymin^2);  % Минимальная скорость
vmax = sqrt(vxmax^2 + vymax^2);  % Максимальная скорость

bin = 5;
E = n * v0^2 / 2;
border = 2 * max(vx);

subplot(1, 3, 1);
x_teor = -border:border;
y_teor = bin * sqrt((1 / (2 * pi)) * (n / E)) * exp(-x_teor.^2 * n / (2 * E));
plot(x_teor, y_teor, 'LineWidth', 5);
hold on;
h = histogram(vx, -border:bin:border, 'Normalization', 'probability');
hold off;
title('Vx');

subplot(1, 3, 2);
x_teor = 0:2 * border;
y_teor = x_teor * bin * (n / E) .* exp(-x_teor.^2 * n / (2 * E));
plot(x_teor, y_teor, 'LineWidth', 5);
hold on;
v = sqrt(vx.^2 + vy.^2);
h2 = histogram(v, 0:bin:2 * border, 'Normalization', 'probability');
hold off;
title('|V|');

s = subplot(1, 3, 3);
x_teor = 0:border^2;
y_teor = 100 * bin * (n / E) .* exp(-x_teor * n / E);
plot(x_teor, y_teor, 'LineWidth', 5);
hold on;
e = (vx.^2 + vy.^2) / 2;
h3 = histogram(e, 0:100 * bin:border^2, 'Normalization', 'probability');
hold off;
title('E');

accum = [];
accum2 = [];
accum3 = [];
t = 0;

while ishghandle(h)
    t = t + dt;
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
    
    v = sqrt(vx.^2 + vy.^2);
    e = (vx.^2 + vy.^2) / 2;
    E = sum(e) / n * 1.67e-27;
    timelabel = ['E = ', num2str(E, '%g'), ' J, T = 2E/3k = ', num2str((2 * E) / (3 * 1.38e-23), '%g')];
    title(s, timelabel);
    
    if t > 3
        accum = [accum, vx];
        accum2 = [accum2, v];
        accum3 = [accum3, e];
        h.Data = accum;
        h2.Data = accum2;
        h3.Data = accum3;
    else
        h.Data = vx;
        h2.Data = v;
        h3.Data = e;
    end
    
    pause(0.00001);
end;
