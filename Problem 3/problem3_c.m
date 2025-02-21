clear all 
close all 
clc

% Main Program
k = 1; 
m = 1;
x0 = 1; 
v0 = 0; 
tend = 20; 
tspan = [0, tend];
dt = 0.01;

y0 = [x0, v0];

% Part (c)
odefun = @(t, y) rhs(t, y, k, m); % Pass parameters using an anonymous function
solution = ode45(odefun, tspan, y0); 
t = linspace(0, tend, 1000); 
y = deval(solution, t); % Evaluate the solution at finer time points

x = y(1, :); 
v = y(2, :);
plotting(t, x, v)

function plotting(t, x, v)
    figure;
    axis equal
    subplot(2, 1, 1);
    plot(t, x, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Displacement (m)');
    title('Harmonic Oscillator (ODE45 - Smoothed Plot)');
    
    subplot(2, 1, 2);
    plot(t, v, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
end

% RHS function
function ydot = rhs(~, y, k, m)
    x = y(1);
    v = y(2);
    
    xdot = v; 
    vdot = -k / m * x; 
    
    ydot = [xdot; vdot]; % Return column vector
end
