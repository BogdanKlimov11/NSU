clear
x = 0.0; p = 0; f = 1;
t = 0; dt = 0.01;
check = 0;

stop = 1;
set(gca,'ButtonDownFcn','stop=0;');
set(0,'units','pixels'); res = get(0,'screensize');
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3])

axis ([0 3.5 0 2], 'equal'); xlim([0 3.5]); ylim([0 2]);
title(['Frequency']);
grid on; xlabel('xmax'); ylabel('freq(xmax)');
h = animatedline(x,f,'Color','black');

for xmax = 0.01:0.01:3.14
    x = xmax;
    while stop && ishghandle(h)
        t = t + dt;
        x = x + p*dt;
        p = p - sin(x)*dt;
        if p > 0 && x < 0
            break;
        end;
    end;
    f = 2*pi/(2*t);
    addpoints(h,xmax,f);
    t = 0;
    drawnow;
    if abs(f-1) > 0.001 && check == 0
        disp(['frequency changes for more than .1% at x = ', num2str(xmax), ' rad']);
        check = 1;
    end;
        
end;
    
    
