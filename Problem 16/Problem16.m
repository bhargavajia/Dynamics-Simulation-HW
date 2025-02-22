% Problem-16

clc;
clear all;
close all;

% Define Parameters
p.G = 6.67 * 1e-11; p.m1 = 1; p.m2 = 1e5;
time_scale = 1e10;

% Define Time Span
r10 = [0; 0]; r20 = [1e5; 0]; v10 = [0; 8.167006795638167e-06]; v20 = [0; 0];
z0 = [r10; r20; v10; v20];

time_period = 2*pi*sqrt(norm(r20-r10)^3/(p.G*max(p.m1,p.m2)));
tstart = 0; tend = time_period; tspan = [tstart tend];

% Finding EoM
rhs = @(t, z) myrhs(z,t,p);

% Solving ODE
options = odeset('AbsTol', 1e-6, 'RelTol', 1e-6);
solution = ode45(rhs, tspan, z0, options);

% Animate the motion
animate(solution, tspan, z0, time_scale)

%% 3 Particles
p.G = 10; p.d = 3; p.m = 1;
r10 = [0; 1.5]; r20 = -1.5*[sind(60); cosd(60)]; r30 = 1.5*[sind(60); -cosd(60)];

v_scalar = sqrt(4*p.G*p.m/p.d);
v10 = [v_scalar; 0]; v20 = v_scalar * [r20(2); -r20(1)]/norm(r20); v30 = v_scalar * [r30(2); -r30(1)]/norm(r30);
z0 = [r10; r20; r30; v10; v20; v30];

time_period = 2*pi*sqrt(p.d^3/(16*p.G*p.m));
time_scale = time_period/10;
tstart = 0; tend = time_period; tspan = [tstart tend];

rhs = @(t, z) myrhs_3particles(z,t,p);

options = odeset('AbsTol', 1e-6, 'RelTol', 1e-6);
solution = ode45(rhs, tspan, z0, options);

animate_3particles(solution, tspan, z0, time_scale);