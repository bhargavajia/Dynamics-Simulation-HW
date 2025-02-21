close all
clear all
clc

tend = 50;
tspan = [0,tend]; % Simulation length

% Initial conditions
r0 = [0;0];
v0 = [1;10];
z0 = [r0;v0];

% Solve the system ODEs
solution = ode45(@airmass, tspan, z0);


% Parameters for animation
t = linspace(0, tend, 100);
z = deval(solution, t);
x = z(1, :); % Extract x values
y = z(2, :); % Extract y values

% Create a figure and plot setup
figure;
h = plot(nan, nan, 'b-', 'LineWidth', 1.5); % Line for trajectory
hold on;
h_point = plot(nan, nan, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 

% Set plot limits (adjust based on expected trajectory)
xlim([min(x) max(x)]);
ylim([min(y) max(y)]);
xlabel('x');
ylabel('y');
title('Trajectory');
grid on;

% Animation loop
for i = 1:length(t)
    set(h, 'XData', x(1:i), 'YData', y(1:i)); % Update trajectory
    set(h_point, 'XData', x(i), 'YData', y(i));
    pause(0.05); % Adjust for desired animation speed
end
