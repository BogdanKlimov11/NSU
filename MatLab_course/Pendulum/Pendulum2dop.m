clear
x = 0.0; p = 1.5; E = p*p/2 + 1 - cos(x); Emax = E; Emin = E; dE = 0;
t = 0; dt = 0.01;
direction = 0;

stop = 1;
set(gca,'ButtonDownFcn','stop=0;');
set(0,'units','pixels'); res = get(0,'screensize');
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3])

axis ([-pi pi 0.8 1.2], 'equal'); xlim([0 1.5]); ylim([0 1.5]);
title(['dE/E(dt)']);
grid on; xlabel('dt'); ylabel('dE/E');
h = animatedline(x,E,'Color','black');

for dt = 0.01:0.02:1.0
    while stop && ishghandle(h)
        t = t + dt;
        x = x + p*dt;
        p = p - sin(x)*dt;
        E = p*p/2 + 1 - cos(x);
        if E > Emax
            Emax = E;
        end;
        if E < Emin
            Emin = E;
        end;
        if x < 0
            direction = 1;
        elseif x > 0 && direction == 1
            break;
        end;
    end;
    dE = Emax - Emin;
    addpoints(h,dt,dE/E);
    direction = 0;
    drawnow;
end;
    
    
