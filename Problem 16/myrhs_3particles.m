function zdot = myrhs_3particles(z,t,p)
    m = p.m; d = p.d; G = p.G;

    r1 = z(1:2);
    r2 = z(3:4);
    r3 = z(5:6);

    v1 = z(7:8);
    v2 = z(9:10);
    v3 = z(11:12);

    a1 = -16*G*m/(d^3) * r1;
    a2 = -16*G*m/(d^3) * r2;
    a3 = -16*G*m/(d^3) * r3;

    zdot(1:2) = v1;
    zdot(3:4) = v2;
    zdot(5:6) = v3;

    zdot(7:8) = a1;
    zdot(9:10) = a2;
    zdot(11:12) = a3;

    zdot = zdot';
end