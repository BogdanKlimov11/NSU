clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2+2, (resY - resY*0.8)/2, resY*0.8, resY*0.8])

global rad lx ly m n;
n = 300;                %���������� �����
dt = .01;               %��� �� �������
tmot = 0;               %������� �����
lx = 200; ly = 200;     %������� ���������
rad(1:n) = 1;           %������� �����
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


h = animatedline(vx,vy); %������ ����
set(h,'LineStyle','none','color','k','Marker','o','MarkerSize',5*rad(1));
axis([-lx, lx, -ly, ly]);
axis('square'); grid on;

while ishghandle(h)
    [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
    clearpoints(h);
    addpoints(h,vx,vy);
    pause(0.000001);               
end;
 
