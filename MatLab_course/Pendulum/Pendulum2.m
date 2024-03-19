clear
x = 0.0; p = 1.5; E = p*p/2 + 1 - cos(x);
t = 0; dt = 0.02;

stop = 1;
set(gca,'ButtonDownFcn','stop=0;');
set(0,'units','pixels'); res = get(0,'screensize');
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3])

subplot (2, 1, 1);
axis ([-pi pi -pi pi ], 'equal'); xlim([-pi pi]); ylim([-pi pi]);
title(['x = ', num2str(x),'   p = ', num2str(p)]);
grid on; xlabel('x'); ylabel('p = x`(t)');
h1 = animatedline(x,p,'Color','black');

subplot (2, 1, 2);
axis ([-pi pi 0.8 1.2], 'equal'); xlim([-pi pi]); ylim([0 2]);
title(['x = ', num2str(x),'   E = ', num2str(E)]);
grid on; xlabel('x'); ylabel('E = p*p/2 + 1 - cos(x)');
h2 = animatedline(x,E,'Color','black');
h2o = animatedline(x,E,'Color','black','Marker','*','MarkerSize',5);

%x = x - p*dt/2;
while stop && ishghandle(h1)
    t = t + dt;
    x = x + p*dt;
    p = p - sin(x)*dt;
    E = p*p/2 + 1 - cos(x);
    addpoints(h1,x,p);
    addpoints(h2,x,E);
    clearpoints(h2o);
    addpoints(h2o,x,E);
    drawnow;
%    pause(0.1)
end;