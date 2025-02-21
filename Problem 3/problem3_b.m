% Main Program
k = 1; 
m = 1;
x0 = 1; 
v0 = 0; 
tend = 20; 
dt = 0.01;

N = floor(tend / dt); 
t = linspace(0, tend, N);

y0 = [x0, v0];

y = euler(@rhs, y0, t, dt, k, m);
x = y(:, 1);
v = y(:, 2);

figure;
axis equal
subplot(2, 1, 1);
plot(t, x, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Harmonic Oscillator (Euler Method)');

subplot(2, 1, 2);
plot(t, v, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');

% Euler function
function y = euler(rhs, y0, t, dt, k, m)
    N = length(t); 
    y = zeros(N, length(y0)); 
    y(1, :) = y0; 
    
    for i = 1:N-1
        ydot = rhs(y(i, :), k, m); 
        y(i+1, :) = y(i, :) + ydot * dt; 
    end
end

% RHS function
function ydot = rhs(y, k, m)
    x = y(1);
    v = y(2);
    
    xdot = v; 
    vdot = -k / m * x; 
    
    ydot = [xdot, vdot]; 
end
