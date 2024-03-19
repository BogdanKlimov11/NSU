clear;
clear global;
cla;
warning off;
global rad lx ly m n;
n=10;	%количество шаров
dt=0.05;             %шаг по времени
tmax=1000.0;             %максимальное времЯ счета
tmot=0;              %текущее времЯ
lx=200; ly=200;      %размеры биллиарда
rad(1:n)=5;	%радиусы шаров
m(1:n)=1;       %массы шаров
g1=0.001;       %ускорение свободного падениЯ
dv=0.2;
%% начальные скорости и координаты из set_random.m
out=set_random(20);  % задаем случайные координаты частиц, 
x=out(1,:);	      %скорости всех направлены по х и равны аргументу	
y=out(2,:);
vx=out(3,:);
vy=out(4,:);
%% вычилсление минимального и максимального значения 
vxmin=min(vx);
vymin=min(vy);
vxmax=max(vx);
vymax=max(vy);
vmin=sqrt(vxmin^2+vymin^2);
vmax=sqrt(vxmax^2+vymax^2);
%% вывод на график 

h=animatedline(x,y); % рисуем шары
set(h,'LineStyle','none','color','b','Marker','o','MarkerSize',2.4*rad(1));
axis([0, lx, 0, ly]);
axis('square');

while (tmot<tmax) && ishghandle(h)
     %tmot=tmot+dt;         % 
   %xin=x; yin=y; vxin=vx; vyin=vy; % 
%   tic
 %  Balls_mex(n,dt,g1,x,y,vx,vy,m,rad,[0 0 lx ly]); %откомпилированный модуль на С длЯ расчета новых координат и скоростей
    [x, y, vx, vy]=BallsF8(n,x,y,vx,vy,dt);	%откомпилированнаЯ функциЯ MATLAB, можно посмотреть текст программы
  clearpoints(h);
  addpoints(h,x,y);
    
   
    pause(0.0001);               
 end;
 
