% Parameters
m = 1;        % Mass of the pendulum bob (kg)
g = 10;       % Gravitational acceleration (m/s^2)
l = 1;        % Length of the pendulum (m)
tspan = [0 10];  % Simulation time (s)

% Initial conditions: [x0, y0, xdot0, ydot0]
theta0 = pi/3;           % Initial angle
x0 = l * cos(theta0);    % Initial x position
y0 = l * sin(theta0);    % Initial y position
xdot0 = 0;               % Initial x velocity
ydot0 = 0;               % Initial y velocity
z0 = [x0; y0; xdot0; ydot0];

% Solve the system using ODE45 with strict tolerances
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[t, sol] = ode45(@(t, z) pendulum_dae(t, z, m, g, l), tspan, z0, options);

% Extract results
x = sol(:,1);
y = sol(:,2);
xdot = sol(:,3);
ydot = sol(:,4);

% Enforce constraint (projection to the circle)
for i = 1:length(x)
    r = sqrt(x(i)^2 + y(i)^2);
    x(i) = x(i) * (l / r);
    y(i) = y(i) * (l / r);
end

% Call plotting function
plot_pendulum_trajectory(x, y, t);

% Call animation function
animate_pendulum(x, y, t);

% Function to plot the pendulum trajectory
function plot_pendulum_trajectory(x, y, t)
    % Plot trajectory of pendulum
    figure;
    plot(x, y, 'b', 'LineWidth', 2); hold on;
    plot(x(1), y(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Initial position
    xlabel('x (m)');
    ylabel('y (m)');
    title('Pendulum Trajectory');
    axis equal; grid on;

    % Plot x and y over time
    figure;
    subplot(2,1,1);
    plot(t, x, 'r', 'LineWidth', 2);
    ylabel('x (m)');
    title('Position Over Time');
    grid on;

    subplot(2,1,2);
    plot(t, y, 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('y (m)');
    grid on;
end

% Function to animate the pendulum
function animate_pendulum(x, y, t)
    % Set up the figure for animation
    figure;
    axis equal;
    axis([-1.5 1.5 -1.5 1.5]); % Adjust axis limits
    grid off;
    hold on;

    % Plot the pendulum bob
    pendulum_bob = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Pendulum bob
    rod = plot([0, 0], [0, 0], 'k', 'LineWidth', 2); % Rod that connects to the bob

    % Animation loop
    for i = 1:length(t)
        % Update pendulum bob position
        set(pendulum_bob, 'XData', x(i), 'YData', y(i));
        
        % Update the rod's position
        set(rod, 'XData', [0, x(i)], 'YData', [0, y(i)]);
        
        % Pause for animation effect
        drawnow;
    end
end

% Function defining the system of DAEs for the pendulum
function zdot = pendulum_dae(~, z, m, g, l)
    x = z(1);
    y = z(2);
    xdot = z(3);
    ydot = z(4);
    
    % Compute theta correctly
    theta = atan2(y, x);  % Corrected

    % Constraint Equations
    A = [m, 0, cos(theta);
         0, m, sin(theta);
         x, y, 0];

    B = [m * g; 0; -(xdot^2 + ydot^2)]; % Constraint equation

    sol = A\B;
    xddot = sol(1);
    yddot = sol(2);
    
    zdot = [xdot; ydot; xddot; yddot];
end
