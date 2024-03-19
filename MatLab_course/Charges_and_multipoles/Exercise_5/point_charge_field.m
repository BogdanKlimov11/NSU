function E=point_charge_field(Q,x0,x)
    if x0 == x 
        E=[NaN NaN NaN];
    else
        R=[x(1)-x0(1),x(2)-x0(2),x(3)-x0(3)];
        E=(Q/dot(R,R))*(R/norm(R));
    end
  %return E  
end