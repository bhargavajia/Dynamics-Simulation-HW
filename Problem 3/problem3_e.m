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
dt = 0.01; % Fixed time step size

% Tolerance values to test
reltol_list = [1, 1e-1, 1e-2, 1e-4, 1e-6]; 
abstol_list = [1, 1e-1, 1e-2, 1e-4, 1e-6]; 

% Store solutions and errors for each tolerance setting
errors = zeros(length(reltol_list), 1); % To store errors for different tolerances
solutions = cell(length(reltol_list), 1); % To store solutions for each tolerance
time_arrays = cell(length(reltol_list), 1);

% Solve with the smallest tolerance first (reltol = 1e-6, abstol = 1e-6)
dt_min = dt; % Fixed dt
N_min = floor(tend / dt_min); 
t_min = linspace(0, tend, N_min);
y0 = [x0, v0];
odefun = @(t, y) rhs(t, y, k, m);
solution_best = ode45(odefun, tspan, y0, odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
t_best = linspace(0, tend, 1000); 
y_best = deval(solution_best, t_best); 
solutions{1} = y_best; 
time_arrays{1} = t_best;

% Loop through each tolerance setting and solve the ODE
for i = 1:length(reltol_list)
    reltol = reltol_list(i); % Choose the appropriate tolerance for the current iteration
    abstol = abstol_list(i); % Corresponding absolute tolerance
    options = odeset('RelTol', reltol, 'AbsTol', abstol);
    
    % Solve ODE for current tolerance settings
    solution = ode45(odefun, tspan, y0, options);
    t = linspace(0, tend, N_min); % Fixed time grid
    y = deval(solution, t); % Finer evaluation of the solution
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

% Plotting the errors vs. tolerance settings
figure;
loglog(reltol_list, errors, '-o', 'LineWidth', 1.5);
xlabel('Relative Tolerance (RelTol)');
ylabel('Error');
title('Error Estimation for Different Tolerances');
grid on;

% Plotting all solutions for comparison
for i = 1:length(reltol_list)
    figure
    axis equal

    subplot(2, 1, 1);
    y = solutions{i};
    t = time_arrays{i};
    plot(t, y(1, :), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Displacement (m)');
    title(sprintf('Harmonic Oscillator for RelTol = %.3e', reltol_list(i)))
    hold on
    
    subplot(2, 1, 2);
    plot(t, y(2, :), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title(sprintf('Harmonic Oscillator for RelTol = %.3e', reltol_list(i)))
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
