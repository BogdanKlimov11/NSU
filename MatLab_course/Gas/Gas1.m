clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2+2, (resY - resY*0.8)/2, resY*0.8, resY*0.8])

global rad lx ly m n;
n = 30;                 %���������� �����
dt = .01;               %��� �� �������
tmot = 0;               %������� �����
lx = 200; ly = 200;     %������� ���������
rad(1:n) = 5;           %������� �����
m(1:n) = 1;             %����� �����
g1 = 0.001;             %��������� ���������� �������
v0 = 100;               %��������� �������� � ���������� �� set_random.m
out = set_random(v0);	%������ ��������� ���������� ������, 
x = out(1,:);           %�������� ���� ���������� �� � � ����� ���������	
y = out(2,:);
vx = out(3,:);
vy = out(4,:);
vxmin = min(vx);        %���������� ������������ � ������������� �������� 
vymin = min(vy);
vxmax = max(vx);
vymax = max(vy);
vmin = sqrt(vxmin^2 + vymin^2);
vmax = sqrt(vxmax^2 + vymax^2);

h = animatedline(x,y); %������ ����
set(h,'LineStyle','none','color','k','Marker','o','MarkerSize',5*rad(1));
axis([0, lx, 0, ly]);
axis('square');

ht = animatedline(x(1), y(1), 'color', 'blue', 'LineWidth', 2);
stop = 1; set(gca,'ButtonDownFcn','vx = -vx; vy = -vy; ht = animatedline(x(1), y(1), ''color'', ''red'', ''LineWidth'', 1);');

t = 0;
while ishghandle(h)
    t = t + dt;
    [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
    clearpoints(h);
    addpoints(h,x,y); addpoints(ht,x(1),y(1));
    pause(0.000001);               
end;
 
