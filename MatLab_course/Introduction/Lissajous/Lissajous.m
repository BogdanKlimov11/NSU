clear;
[a1,a2,w1,w2,fi1,fi2] = inputparam();
t = 0;
dt = 0.03;
global stop;

x = a1*cos(w1*t + fi1);
y = a2*sin(w2*t + fi2);
h = animatedline(x,y,'Color','black');
ho = animatedline(x,y,'Color','r','Marker','*','MarkerSize',5);
axis([-a1 a1 -a2 a2],'equal');
title('Lissajous');
grid on;
xlabel('X');
ylabel('Y');
stop = 1;
timelabel = ['t = ',num2str(t,'%.2f'),' s'];
stoplabel = ['stop = ',num2str(stop)];
ht = text(-a1,a2*1.05,timelabel);
hs = text(a1,a2*1.05,stoplabel);
set(gca,'ButtonDownFcn','stop = mod(stop+1,2); [t] = animate(t, dt, h, ho, ht, hs);')

[t] = animate(t, dt, h, ho, ht, hs);

