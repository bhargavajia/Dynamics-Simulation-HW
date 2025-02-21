close all
clear all
clc

tend = 10;
tspan = [0,tend]; % length

% Initial conditions
x0 = 1; y0 = 1; vx0 = 1; vy0 = 1;
r0 = [x0;y0];
v0 = [vx0;vy0];
z0 = [r0;v0];

% Solve the system ODEs
solution = ode45(@rhs, tspan, z0);
t = linspace(0, tend, 100);
z = deval(solution, t);
x = z(1, :); % Extract x values
y = z(2, :); % Extract y values

solution = ode45(@rhs, tspan, z0);
t = linspace(0, tend, 100);
z = deval(solution, t);
x = z(1, :); % Extract x values
y = z(2, :); % Extract y values

% Plotting trajectory
axis equal
plot(x, y, 'b', 'LineWidth', 1.5)
hold on;

% Marking start and end points
plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8); % Start point in green
text(x(1), y(1), 'Start', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % End point in red
text(x(end), y(end), 'End', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

xlabel('X axis');
ylabel('Y axis');
title('Trajectory of mass with spring L0 = 0, g = 0 and v0 along r0');
hold off;

% figure;
% axis equal
% subplot(2, 1, 1);
% plot(t, x, 'b', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('X Position (m)');
% title('Position vs time for g = 0 and v0 = 0');
% 
% subplot(2, 1, 2);
% plot(t, y, 'r', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Y Position (m)');



% State space form of EoM
function zdot = rhs(t,z)
    r = z(1:2);
    v = z(3:4);
    
    i = [1; 0]; j = [0; 1];
    m = 1;
    k = 1;
    g = 0;
    L0 = 0;
    
    Fg = - m * g * j;
    Fspring = k * (norm(r)-L0) * (-r)/ norm(r);
    Ftot = Fg + Fspring;
    
    rdot = v;
    vdot = Ftot/m;
    
    zdot = [rdot; vdot];
end

% For L0 = 0, the spring is a 0 length spring and the trajectory follows an
% elliptical path due to the spring force, gravity.

% For L0 = 0, g = 0, and v0 in the direction of r0, the spring undergoes harmonic motion about the
% origin in the line of initial r0. 