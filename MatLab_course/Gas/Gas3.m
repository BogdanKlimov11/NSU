clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX*.1, (resY - resY*0.8)/2, resX*.9, resY*.8])

global rad lx ly m n;
n = 100;                %количество шаров
dt = .1;                %шаг по времени
tmot = 0;               %текущее время
lx = 200; ly = 200;     %размеры биллиарда
rad(1:n) = 1;           %радиусы шаров
m(1:n) = 1;             %массы шаров
g1 = 0.001;             %ускорение свободного падения
v0 = 100;               %начальные скорости и координаты из set_random.m
out = set_random(v0);	%задаем случайные координаты частиц, 
x = out(1,:);           %скорости всех направлены по х и равны аргументу	
y = out(2,:);
vx = out(3,:);
vy = out(4,:);
vxmin = min(vx);        %вычисление минимального и максимального значения 
vymin = min(vy);
vxmax = max(vx);
vymax = max(vy);
vmin = sqrt(vxmin^2 + vymin^2);
vmax = sqrt(vxmax^2 + vymax^2);

bin = 5; E = n*v0*v0/2; border = 2*max(vx);

subplot(1,3,1)
x_teor = -border:border;
y_teor = bin*sqrt((1/(2*pi))*(n/E))*exp(-x_teor.^2*n/(2*E));
plot(x_teor, y_teor, 'LineWidth', 5); hold on;
h = histogram(vx, -border:bin:border, 'Normalization', 'probability'); hold off;
title('Vx');

subplot(1,3,2)
x_teor = 0:2*border;
y_teor = x_teor*bin*(n/E).*exp(-x_teor.^2*n/(2*E));
plot(x_teor, y_teor, 'LineWidth', 5); hold on;
v = (vx.^(2) + vy.^2).^(1/2);
h2 = histogram(v, 0:bin:2*border, 'Normalization', 'probability'); hold off;
title('|V|');

subplot(1,3,3)
x_teor = 0:border^2;
y_teor = 100*bin*(n/E).*exp(-x_teor*n/E);
plot(x_teor, y_teor, 'LineWidth', 5); hold on;
e = (vx.^(2) + vy.^2)/2;
h3 = histogram(e, 0:100*bin:border^2, 'Normalization', 'probability'); hold off;
title('E');

accum = []; accum2 = []; accum3 = []; t = 0;
while ishghandle(h)
    t = t + dt;
    [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
    v = (vx.^(2) + vy.^2).^(1/2);
    e = (vx.^(2) + vy.^2)/2;
    if (t > 3) %после трех "секунд" включается накопление
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
    pause(.00001); 
end;
 
