clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2+2, (resY - resY*0.8)/2, resY*0.8, resY*0.8])

global rad lx ly m n;
n = 300;                %количество шаров
dt = .01;               %шаг по времени
tmot = 0;               %текущее время
lx = 200; ly = 200;     %размеры биллиарда
rad(1:n) = 1;           %радиусы шаров
m(1:n) = 1;             %массы шаров
g1 = 0.001;             %ускорение свободного падениЯ
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


h = animatedline(vx,vy); %рисуем шары
set(h,'LineStyle','none','color','k','Marker','o','MarkerSize',5*rad(1));
axis([-lx, lx, -ly, ly]);
axis('square'); grid on;

while ishghandle(h)
    [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
    clearpoints(h);
    addpoints(h,vx,vy);
    pause(0.000001);               
end;
 
