function zdot = airmass(t,z)
r = z(1:2);
v = z(3:4);

i = [1; 0]; j = [0; 1];
m = 1;
c = 1;
g = 9.8;
F = 100;

Fg = - m * g * j;
Fdrag = - c * v;
Fthrust = F * v / norm(v);
Ftot = Fg + Fdrag + Fthrust;

rdot = v;
vdot = Ftot/m;

zdot = [rdot; vdot];