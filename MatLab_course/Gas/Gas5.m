 clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [(resX-resY*1.6)+2, (resY - resY*0.8)/2, resY*1.6, resY*0.8])

global rad lx ly m n;
n = 20;                %количество шаров
dt = .1;                %шаг по времени
tmot = 0;               %текущее время
lx = 200; ly = 200;     %размеры биллиарда
rad(1:n) = 10;           %радиусы шаров
m(1:n) = 1;             %массы шаров
g1 = 0.001;             %ускорение свободного падения
v0 = 10;               %начальные скорости и координаты из set_random.m
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

coordsX = [0,lx,lx,0, 0];
coordsY = [0,0,ly,ly, 0];

subplot(1,2,1)
p = animatedline(coordsX, coordsY, 'Color', 'b');
h = animatedline(x,y);
set(h,'LineStyle','none','color','k','Marker','o','MarkerSize',2.1*rad(1));
axis([0, 2*lx, -ly/2, 3*ly/2]);
axis('square');

s = subplot(1,2,2);
E = 0; t = 0;
for i = 1:n
    E = E + vx(i)^2/2 + vy(i)^2/2;
end;
%e = E/n;

hE = animatedline(t, E, 'color', 'k');
%he = animatedline(t, 0, 'color', 'blue', 'LineWidth', 2);
timelabel = ['t = ',num2str(t,'%.2f'),' s'];
t_title = title(timelabel);

U = 50; T = 100/U; T_counter = 0; counter = 0; vx = vx - U;
while ishghandle(h)
    if (T_counter >= T)
        T_counter = 0; U = -U;
        vx = vx - 2*U;
    end;
    coordsX = coordsX + U*dt; clearpoints(p); addpoints(p,coordsX,coordsY);
    t = t + dt; T_counter = T_counter + dt;
    E = 0;
    [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
    for i = 1:n
        E = E + (vx(i) + U)^2/2 + vy(i)^2/2;
    end;
    %e = E/n;
    clearpoints(h); addpoints(h,(x+coordsX(1)),y);
    addpoints(hE,t,E); %addpoints(he,t,e);
    timelabel = ['t = ',num2str(t,'%.2f'),' s'];
    title(s,timelabel)
    pause(.00001);
end;
 
