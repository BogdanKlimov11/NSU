clear;
clear global;

set(0, 'units', 'pixels'); 
res = get(0, 'screensize'); 
resX = res(3); 
resY = res(4);
set(gcf, 'position', [resX / 2 + 2, (resY - resY * 0.8) / 2, resY * 0.8, resY * 0.8])

global rad lx ly m n;

n = 100;  % Количество частиц
dt = 0.1;  % Шаг по времени
tmot = 0;  % Текущее время
lx = 200; ly = 200;  % Размеры области
rad(1:n) = 1;  % Радиусы частиц
m(1:n) = 1;  % Массы частиц
g1 = 0.001;  % Ускорение гравитации
v0 = 100;  % Начальная скорость и коэффициенты из Set_random.m
out = set_random(v0);  % Задаём случайные начальные координаты частиц, 
x = out(1, :);  % Скорости всех частиц по осям x и y
y = out(2, :);
vx = out(3, :);
vy = out(4, :);

% Минимальная и максимальная скорость по осям x и y
vxmin = min(vx);
vymin = min(vy);
vxmax = max(vx);
vymax = max(vy);

vmin = sqrt(vxmin^2 + vymin^2);
vmax = sqrt(vxmax^2 + vymax^2);

bin = 5; 
border = sqrt(lx^2 + ly^2);

R = []; 
accum = []; 
t = 0;
for i = 1:n
    for j = (i + 1):n
        r = sqrt((x(j) - x(i))^2 + (y(j) - y(i))^2);
        R = [R r];
    end;
end;

h = histogram(R, 0:bin:border, 'Normalization', 'probability');
timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
ht = text(-border * 0.13, 0, timelabel);

while ishghandle(h)
    t = t + dt;
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
    for i = 1:n
        for j = (i + 1):n
            r = sqrt((x(j) - x(i))^2 + (y(j) - y(i))^2);
            R = [R r];
        end;
    end;
    if (t > 20) % После 20 "секунд" происходит накопление
        accum = [accum R];
        h.Data = accum;
    else
        h.Data = R;
    end;
    timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
    set(ht, 'String', timelabel);
    R = [];
    pause(0.00001); 
end;
