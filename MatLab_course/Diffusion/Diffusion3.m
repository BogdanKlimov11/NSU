% Стартовая программа для демонстрации случайных блужданий
n =500; % Число частиц
dh =.02; % Параметр случайного распределения
% Задание вектор-столбцов координат точек
y =1:n;
y=y'; x =zeros(size(y));
subplot(1,2,1);
h=plot(x,y,'k.'); % Вывод начального положения точек
axis([-2 2 0 n+1 ]); % Задание осей
% Определение режима перерисовки и размера точек
set(h,'EraseMode','background','MarkerSize',3);
pause(1); % Пауза для вывода графика на экран
i=0; % Начальное значение
subplot(1,2,2);
h1 = line;
h2 = line;
set(h1,'LineStyle','.');
set(h2,'LineStyle','.');
h3 = line;
h4 = line;
set(h3,'Color','r');
set(h4,'Color','r');
set(h3,'YData',[0],'XData',[0]);
set(h4,'YData',[0],'XData',[0]);
g1=0;
g2=0;

while i < 1000 % neБесконечный цикл
i=i+1;
x=x+dh*(2*rand(n,1)-1); % Случайные смещения x-координаты
% Смена координат точек на рисунке
set(h,'XData',x,'YData',y,'Color','k');
a=0;
c=0;
for b=1:n;
  a=a+x(b);
  c=c+x(b)*x(b);
end;
a=a/n;
c=c/n;
g1(i)=a;
g2(i)=c;

set(h1,'XData',(1:i),'YData',g1);
set(h2,'XData',(1:i),'YData',g2);

pause(0.01);
end;

xx=(1:i);
od=ones(size(g2));
s(1,1) = od*(od');
s(2,1) = od*(xx');
s(1,2) = s(2,1);
s(2,2) = xx*(xx');
p = [od*(g2'); xx*(g2')];
a = s \ p;

set(h3,'YData',[a(1) a(1)+(i)*a(2)],'XData', [0 i]);

xx=(1:i);
od=ones(size(g1));
s(1,1) = od*(od');
s(2,1) = od*(xx');
s(1,2) = s(2,1);
s(2,2) = xx*(xx');
p = [od*(g1'); xx*(g1')];
a = s \ p;

set(h4,'YData',[a(1) a(1)+(i)*a(2)],'XData', [0 i]);
