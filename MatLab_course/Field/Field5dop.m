%Солнце, Земля, луна
clear;
alpha = 10; beta = .5;
dt = 0.05;
x1 = 0; y1 = 10; R1 = [x1 y1]; V1 = [1.3 0];
x2 = 1; y2 = 10; R2 = [x2 y2]; V2 = [1.3 .7];

stop = 1; set(gca,'ButtonDownFcn','stop = 0;');
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, resY/2-resY/4, resX/3, resY/2])

h1 = animatedline(x1, y1, 'color', 'k');
hp1 = animatedline(x1, y1, 'color', 'k', 'marker', 'o', 'markersize', 5);
h2 = animatedline(x2, y2, 'color', 'b');
hp2 = animatedline(x2, y2, 'color', 'b', 'marker', 'o', 'markersize', 5);
hS = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);
g = 60; axis([-g/2 g/2 -g g/3]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

R1 = R1 - V1*dt/2; R2 = R2 - V2*dt/2;
while ishghandle(h1) && stop
    clearpoints(hp1); clearpoints(hp2);
    R1 = R1 + V1*dt; R2 = R2 + V2*dt; delta_R = R2 - R1;
    r1 = sqrt(R1*R1'); r2 = sqrt(R2*R2'); delta_r = sqrt(delta_R*delta_R');
    A1 = -alpha*R1/r1^3 - beta*(R1-R2)/delta_r^3*0;
    A2 = -alpha*R2/r2^3 - beta*delta_R/delta_r^3;
    V1 = V1 + A1*dt; x1 = R1(1); y1 = R1(2);
    V2 = V2 + A2*dt; x2 = R2(1); y2 = R2(2);
    addpoints(h1, x1, y1); addpoints(hp1, x1, y1);
    addpoints(h2, x2, y2); addpoints(hp2, x2, y2);
    drawnow
end