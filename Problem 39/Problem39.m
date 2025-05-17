clc
clear all 
close all

% Two Skates problem

% Parameters
p.m = 10; p.I = 5; p.d1 = 0.2; p.d2 = 0.3; 
p.theta1 = deg2rad(30); p.theta2 = deg2rad(60);

% Initial Conditions and timespan
tspan = [0 20];
x0 = 1; y0 = 1; theta0 = 0.1; thetadot0 = 0.1;
% Find initial velocities that satisfy the constraint equation
[xdot0, ydot0] = init_conds(thetadot0, theta0, p.theta1, p.theta2, p.d1, p.d2);
z0 = [x0; y0; theta0; xdot0; ydot0; thetadot0];


% Solve EoMs
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% solve using DAE
% sol = ode45(@(t, z) DAE(z,t,p), tspan, z0, options);

% solve using Lagrange
sol = ode45(@(t, z) Lagrange(z,t,p), tspan, z0, options);


animate(sol, p)


function zdot = DAE(z, t, p)
    [A, B] = DAE_matrices(z, [p.m; p.I; p.d1; p.d2; p.theta1; p.theta2]);
    sol = A \ B;
    zdot = [z(4); z(5); z(6); sol(1); sol(2); sol(3)];
end


function zdot = Lagrange(z, t, p)
    [A, B] = Lagrange_matrices(z, [p.m; p.I; p.d1; p.d2; p.theta1; p.theta2]);
    sol = A \ B;
    zdot = [z(4); z(5); z(6); sol(1); sol(2); sol(3)];
end




    



