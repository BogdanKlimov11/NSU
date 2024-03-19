clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [(resX-resY*1.6)+2, (resY - resY*0.8)/2, resY*1.6, resY*0.8])

global rad lx ly m n;
n = 100;                %���������� �����
dt = .1;                %��� �� �������
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

coordsX = [0,lx,lx,0, 0];
coordsY = [0,0,ly,ly, 0];

subplot(1,2,1)
q = quiver(100,100,0,0, 'Color', 'b', 'LineWidth', 5, 'MaxHeadSize', 10); hold on;
h = animatedline(x,y); hold off;
set(h,'LineStyle','none','color','k','Marker','o','MarkerSize',5*rad(1));
axis([0, lx, 0, ly]);
axis('square');

s = subplot(1,2,2);
E = 0; t = 0;
for i = 1:n
    E = E + vx(i)^2/2 + vy(i)^2/2;
end;
hE = animatedline(t, E, 'color', 'k');
timelabel = ['t = ',num2str(t,'%.2f'),' s'];
t_title = title(timelabel);

T = 5; T_counter = 0; counter = 0;
while ishghandle(h)
    if (t > 10) && (T_counter >= T) % 10 ������ ���������, ����� ������ 5 ������ ������
        Ux = sum(vx)/n; Uy = sum(vy)/n; % �������� ������ ����
        set(q, 'Udata', 5*Ux, 'Vdata', 5*Uy, 'Visible', 'on');
        vx = vx - Ux; vy = vy - Uy; % ����������� ��. ����� �� �������, ����� �������� �������� ��
        T_counter = 0; pause(1);
        set(q, 'Visible', 'off');
    end;
    t = t + dt; T_counter = T_counter + dt;
    E = 0;
    [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
    for i = 1:n
        E = E + (vx(i))^2/2 + vy(i)^2/2;
    end;
    clearpoints(h); addpoints(h,x,y);
    addpoints(hE,t,E);
    timelabel = ['t = ',num2str(t,'%.2f'),' s'];
    title(s,timelabel);
    pause(.00001);
end;
 
