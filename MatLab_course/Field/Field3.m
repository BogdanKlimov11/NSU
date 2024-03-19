clear;
alpha = 1; beta = .1*0;
dt = 0.05;
x = 1.5; y = 0; z = 0; R = [x y z];
V = [0 .6 0];
Az = [0 0 .05];

stop = 1; %set(gca ,'ButtonDownFcn','stop = 0;');
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, resY/2-resY/4, resX/3, resY/2])
h = animatedline(x, y, z, 'color', 'k'); %Траектория
hp = animatedline(x, y, z, 'color', 'b', 'marker', 'o', 'markersize', 5); %Планета
hS = line(0, 0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7); %Центр поля
h_center = animatedline(0, 0, 0, 'linestyle', 'none', 'color', 'm', 'marker', 'square', 'markersize', 10);
clearpoints(h_center);
g = 2; axis([-g g -g g -g g]); axis square; box on;
%set(gca, 'xtick', [], 'ytick', [], 'ztick', []);
R = R - V*dt/2;

r = sqrt(R*R'); r1 = r; r2 = r; Xs = []; Ys = []; Zs = []; i = 0;
while ishghandle(h) && stop
clearpoints(hp);
R = R + V*dt; %Радиус-вектор
r2 = r1; r1 = r;
r = sqrt(R*R');
Xs = [Xs R(1)]; Ys = [Ys R(2)]; Zs = [Zs R(3)]; 
if (r2 - r1)*(r1 - r) < 0
    if i == 1
        addpoints(h_center, mean(Xs), mean(Ys), mean(Zs));
        Xs = []; Ys = []; Zs = []; i = 0;
    else
        i = i + 1;
    end;
end;
A = -alpha*R/r^3 - 2*beta*R/r^4 + Az; %Ускорение
V = V + A*dt; x = R(1); y = R(2); z = R(3);
%addpoints(h, x, y, z);
addpoints(hp, x, y, z);
drawnow
end