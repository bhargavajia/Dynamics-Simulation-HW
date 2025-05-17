clc
clear 
close all

% Parameters
p.l1 = 1; p.l2 = 1; p.l3 = 1; 
p.d1 = 0.5; p.d2 = 0.5; p.d3 = 0.5;
p.I1 = 1; p.I2 = 2; p.I3 = 3;
p.m1 = 2; p.m2 = 3; p.m3 = 5;

% Initial Conditions and timespan
tspan = [0 10];
theta1_0 = deg2rad(15); theta2_0 = deg2rad(45); theta3_0 = deg2rad(30);
thetadot1_0 = 0.05; thetadot2_0 = 0.1; thetadot3_0 = 0.06; 

z0 = [theta1_0; theta2_0; theta3_0; thetadot1_0; thetadot2_0; thetadot3_0];


% Solve EoMs
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);

% solve using NE
sol = ode45(@(t, z) NE(z,t,p), tspan, z0, options);

% solve using Lagrange
% sol = ode45(@(t, z) Lagrange(z,t,p), tspan, z0, options);

% solve using DAE
% sol = ode45(@(t, z) DAE(z,t,p), tspan, z0, options);

energy_conservation_plot(sol,p)
animate(sol, p)


function zdot = NE(z, t, p)
    [A, B] = NE_matrices(z, [p.l1; p.l2; p.l3; p.d1; p.d2; p.d3; ...
                              p.I1; p.I2; p.I3; p.m1; p.m2; p.m3]);                    
    sol = A \ B;
    zdot = [z(4); z(5); z(6); sol(1); sol(2); sol(3)];
end


function zdot = Lagrange(z, t, p)
    [A, B] = Lagrange_matrices(z, [p.l1; p.l2; p.l3; p.d1; p.d2; p.d3; ...
                                   p.I1; p.I2; p.I3; p.m1; p.m2; p.m3]);
    sol = A \ B;
    zdot = [z(4); z(5); z(6); sol(1); sol(2); sol(3)];
end


function zdot = DAE(z, t, p)
    [A, B] = DAE_matrices(z, [p.l1; p.l2; p.l3; p.d1; p.d2; p.d3; ...
                              p.I1; p.I2; p.I3; p.m1; p.m2; p.m3]);
    sol = A \ B;
    zdot = [z(4); z(5); z(6); sol(1); sol(2); sol(3)];
end

