function [x1,x2] = korni(a, b, c)
if(a ~= 0)
    D = b*b-4*a*c;
    if(D < 0)
        x1 = NaN;
        x2 = NaN;
    else
        x1 = (-b-sqrt(D))/(2*a);
        x2 = (-b+sqrt(D))/(2*a);
        fprintf('%g %g\n', x1, x2);
    end
else
    if(b == 0)
        if(c == 0)
            x1 = NaN;
            x2 = NaN;
        else
            x1 = NaN;
            x2 = NaN;
        end
    else
        x1 = -c/b;
        x2 = NaN;
    end
end