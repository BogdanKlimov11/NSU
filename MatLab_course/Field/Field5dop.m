% Солнце, Земля, Луна
clear;

alpha = 10;
beta = 0.5;
dt = 0.05;

x1 = 0; y1 = 10; r1 = [x1 y1]; v1 = [1.3 0];
x2 = 1; y2 = 10; r2 = [x2 y2]; v2 = [1.3 0.7];

stop = 1;
set(gca, 'ButtonDownFcn', 'stop = 0;');
set(0, 'units', 'pixels');
res = get(0, 'screensize');
res_x = res(3);
res_y = res(4);
set(gcf, 'position', [res_x / 2 - res_x / 6, res_y / 2 - res_y / 4, res_x / 3, res_y / 2]);

h1 = animatedline(x1, y1, 'color', 'k');
hp1 = animatedline(x1, y1, 'color', 'k', 'marker', 'o', 'markersize', 5);
h2 = animatedline(x2, y2, 'color', 'b');
hp2 = animatedline(x2, y2, 'color', 'b', 'marker', 'o', 'markersize', 5);
h_s = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);

g = 60;
axis([-g / 2 g / 2 -g g / 3]);
axis square;
box on;
set(gca, 'xtick', [], 'ytick', []);

r1 = r1 - v1 * dt / 2;
r2 = r2 - v2 * dt / 2;

while ishghandle(h1) && stop
    clearpoints(hp1);
    clearpoints(hp2);
    
    r1 = r1 + v1 * dt;
    r2 = r2 + v2 * dt;
    delta_r = r2 - r1;
    
    dist_r1 = sqrt(r1 * r1');
    dist_r2 = sqrt(r2 * r2');
    delta_dist = sqrt(delta_r * delta_r');
    
    a1 = -alpha * r1 / dist_r1^3 - beta * (r1 - r2) / delta_dist^3 * 0;
    a2 = -alpha * r2 / dist_r2^3 - beta * delta_r / delta_dist^3;
    
    v1 = v1 + a1 * dt;
    x1 = r1(1);
    y1 = r1(2);
    
    v2 = v2 + a2 * dt;
    x2 = r2(1);
    y2 = r2(2);
    
    addpoints(h1, x1, y1);
    addpoints(hp1, x1, y1);
    addpoints(h2, x2, y2);
    addpoints(hp2, x2, y2);
    
    drawnow;
end
