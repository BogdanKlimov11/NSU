clear; clear global;
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2+2, (resY - resY*0.8)/2, resY*0.8, resY*0.8])

global rad lx ly m n;
n = 30;                 %���������� �����
dt = .1;               %��� �� �������
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

s = subplot(1,1,1)
h = animatedline(0, 0, 'color','k'); clearpoints(h);
grid on;
t = 0; 

step = 100; N = step;
for n = 100:-1:10
    n = n
    check = 0;
    rad = []; rad(1:n) = 5;
    m = []; m(1:n) = 1;
    out = set_random(v0);
    x = out(1,:);
    y = out(2,:);
    vx = out(3,:);
    vy = out(4,:);
    while check == 0 && ishghandle(h)
        counter = 0; A = []; t = 0;
        while ishghandle(h) && counter < N
            t = t + dt;
            counter = counter + 1;
            [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
            A = [A (x(1)^2 + y(1)^2)];
            pause(0.000001);
            timelabel = ['t = ',num2str(t,'%.2f'),' s'];
            title(s,timelabel)
        end;
        vx = -vx; vy = -vy; % ����������
        while ishghandle(h) && counter > 1
            t = t + dt;
            counter = counter - 1;
            [x, y, vx, vy] = BallsF8(n,x,y,vx,vy,dt);
            delta = (x(1)^2 + y(1)^2) - A(counter);
            if (delta > 0.01)
                delta = delta
                addpoints(h, n, (N-counter)*dt);
                check = 1;
                break;
            end;
            pause(0.000001);
            timelabel = ['t = ',num2str(t,'%.2f'),' s'];
            title(s,timelabel)
        end;
        if check == 0
            N = N + step % ����� ����, ����� ������� �� �������
        end;
    end
end;
