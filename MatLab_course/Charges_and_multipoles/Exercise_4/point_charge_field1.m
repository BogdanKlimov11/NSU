function E = point_charge_field1(Q, q_crds, crds)
    r = (crds-q_crds);
    if r == 0
        E = 0;
    else
        E = Q/((r(1)^2+r(2)^2+r(3)^2)^(3/2))*r;
    end
end

