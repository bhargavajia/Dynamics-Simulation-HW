% Parameters
k = 1; 
m = 1;
x0 = 1;                    
v0 = 0;                    
tend = 20;               
dt = 0.01;           

% Initialize variables
N = floor(tend / dt);    
t = linspace(0, tend, N);
x = zeros(1, N);          
v = zeros(1, N);           

% Initial conditions
x(1) = x0;
v(1) = v0;

% Euler method 
for i = 1:N-1
    xdot = v(i);                
    vdot = -k / m * x(i); 
    % Update x and v
    x(i+1) = x(i) + xdot * dt;
    v(i+1) = v(i) + vdot * dt;
end

% Plot the results
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
