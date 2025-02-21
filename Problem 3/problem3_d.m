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
dt_list = [2, 1, 0.1, 0.01, 0.001]; % List of different time steps to test

% Store solutions and errors for each step size
errors = zeros(length(dt_list)-1, 1); % To store error between successive step sizes
solutions = cell(length(dt_list), 1); % To store solutions for each dt
time_arrays = cell(length(dt_list), 1);

% Solve with the smallest step size first (dt_min)
dt_min = dt_list(end); % Smallest dt
N_min = floor(tend / dt_min); 
t_min = linspace(0, tend, N_min);
y0 = [x0, v0];
odefun = @(t, y) rhs(t, y, k, m);
solution_best = ode45(odefun, tspan, y0);
t_best = linspace(0, tend, 1000); 
y_best = deval(solution_best, t_best); 
solutions{end} = y_best; 
time_arrays{end} = t_best;

% Loop through each dt and solve the ODE
for i = 1:length(dt_list)-1
    dt = dt_list(i);
    N = floor(tend / dt); 
    t = linspace(0, tend, N);
    y0 = [x0, v0];
    
    % Solve ODE for current dt
    solution = ode45(odefun, tspan, y0);
    y = deval(solution, t); % Finer evaluation of the solution on the same time grid as best solution
    solutions{i} = y;
    time_arrays{i} = t;
    
    x_best = y_best(1, :);
    v_best = y_best(2, :);
    
    % Current solution
    x_current = y(1, :);
    v_current = y(2, :);
    x_current = interp1(t, x_current, t_best, 'pchip'); 
    v_current = interp1(t, v_current, t_best, 'pchip');
    
    % Compute error
    error_x = mean(abs(x_current - x_best));
    error_v = mean(abs(v_current - v_best));
    
    errors(i) = max(error_x, error_v); % Store the maximum of displacement and velocity errors
end

% Plotting the errors vs. step size
figure;
loglog(dt_list(1:end-1), errors, '-o', 'LineWidth', 1.5);
xlabel('Step Size (dt)');
ylabel('Error');
title('Error Estimation for Different Step Sizes');
grid on;

% Plotting all solutions for comparison
for i = 1:length(dt_list)
    figure
    axis equal

    subplot(2, 1, 1);
    y = solutions{i};
    t = time_arrays{i};
    plot(t, y(1, :), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Displacement (m)');
    title(sprintf('Harmonic Oscillator for dt = %.3f', dt_list(i)))
    hold on
    
    subplot(2, 1, 2);
    plot(t, y(2, :), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title(sprintf('Harmonic Oscillator for dt = %.3f', dt_list(i)))
    hold off
end

% RHS function
function rhs = rhs(~, y, k, m)
    x = y(1);
    v = y(2);
    
    xdot = v; 
    vdot = -k / m * x; 
    
    rhs = [xdot; vdot];
end
