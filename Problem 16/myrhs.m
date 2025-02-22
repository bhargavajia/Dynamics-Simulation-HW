function zdot = myrhs(z,t,p)
    m1 = p.m1; m2 = p.m2; G = p.G;

    r1 = z(1:2);
    r2 = z(3:4);
    v1 = z(5:6);
    v2 = z(7:8);

    r = r2-r1;
    r_norm = norm(r);
    
    a1 = G * m2 * r / r_norm^3;
    a2 = - G * m1 * r / r_norm^3;

    zdot(1:2) = v1;
    zdot(3:4) = v2;
    zdot(5:6) = a1;
    zdot(7:8) = a2;

    zdot = zdot';
end