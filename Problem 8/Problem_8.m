clear all 
close all 
clc

% Main Program
r0 = 1;
theta0 = pi/6;
rdot0 = 5;
thetadot0 = 0;
z0 = [r0; theta0; rdot0; thetadot0];

tend = 10; 
tspan = [0, tend];

solution = ode45(@rhs, tspan, z0); 
t = linspace(0, tend, 1000); 
z = deval(solution, t) % Evaluate the solution at finer time points

r = z(1, :); 
theta = z(2, :);

figure;
title("Plot of numerical solution")
x = r.*cos(theta);
y = r.*sin(theta);
plot(x,y, 'LineWidth', 2)
xlabel("X axis")
ylabel("Y axis")
axis equal
title("Trajectory of numerical solution")


rdot = z(3, :); 
thetadot = z(4, :);

figure;
title("Plot of numerical solution")
plot(t,rdot, 'LineWidth', 2)
ylabel("Speed (m/s)")
xlabel("Time (s)")
axis equal
title("Velocity with time")

% RHS function
function zdot = rhs(t, z)
    r = z(1);
    theta = z(2);
    rdot = z(3);
    thetadot = z(4);
    
    rddot = r * thetadot^2; 
    thetaddot = -2*rdot*thetadot/r;
    
    zdot = [rdot; thetadot; rddot; thetaddot]; 
end