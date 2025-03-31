% Parameters
m = 1;        % Mass of each dumbbell component
l = 1;        % Distance between the two masses (m)
tspan = [0 10];  % Simulation time (s)

% Initial conditions: [x1, y1, x2, y2, xdot1, ydot1, xdot2, ydot2]
r10 = [-l/2; 0];
r20 = [l/2; 0];
v10 = [0; -1];
v20 = [0; 1];

z0 = [r10; r20; v10; v20];

% Solve the system using ODE45 with strict tolerances
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, sol] = ode45(@(t, z) dumbbell_dae(t, z, m, l), tspan, z0, options);

% Extract results
x1 = sol(:,1);
y1 = sol(:,2);
x2 = sol(:,3);
y2 = sol(:,4);
x1dot = sol(:,5);
y1dot = sol(:,6);
x2dot = sol(:,7);
y2dot = sol(:,8);

% Compute constraint violation
distance_error = sqrt((x2 - x1).^2 + (y2 - y1).^2) - l;

% Compute kinetic energy
KE = 0.5 * m * (x1dot.^2 + y1dot.^2 + x2dot.^2 + y2dot.^2);
KE_error = KE - KE(1); % Deviation from initial energy

% Compute angular momentum conservation
L = m * (x1 .* y1dot - y1 .* x1dot + x2 .* y2dot - y2 .* x2dot);
L_error = L - L(1);

% Compute linear momentum conservation
Px = m * (x1dot + x2dot);
Py = m * (y1dot + y2dot);
Px_error = Px - Px(1);
Py_error = Py - Py(1);

% Call plotting function
plot_dumbbell_trajectory(x1, y1, x2, y2, t);
plot_numerical_errors(t, distance_error, KE_error, L_error, Px_error, Py_error);

% Call animation function
animate_dumbbell(x1, y1, x2, y2, t);

% Function to plot numerical errors
function plot_numerical_errors(t, distance_error, KE_error, L_error, Px_error, Py_error)
    figure;
    subplot(3,1,1);
    plot(t, distance_error, 'k', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Constraint Violation');
    title('Constraint Violation Over Time'); grid on; axis equal;

    subplot(3,1,2);
    plot(t, KE_error, 'r', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Energy Error');
    title('Energy Conservation Error'); grid on; axis equal;

    subplot(3,1,3);
    plot(t, L_error, 'b', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Angular Momentum Error');
    title('Angular Momentum Conservation Error'); grid on; axis equal;

    figure;
    subplot(2,1,1);
    plot(t, Px_error, 'm', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Px Error');
    title('Linear Momentum Conservation (X)'); grid on; axis equal;

    subplot(2,1,2);
    plot(t, Py_error, 'c', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Py Error');
    title('Linear Momentum Conservation (Y)'); grid on; axis equal;
end

% Function to plot the dumbbell trajectory
function plot_dumbbell_trajectory(x1, y1, x2, y2, t)
    figure;
    plot(x1, y1, 'r', 'LineWidth', 2); hold on;
    plot(x2, y2, 'b', 'LineWidth', 2);
    legend('Mass 1', 'Mass 2');
    xlabel('x (m)'); ylabel('y (m)');
    title('Dumbbell Trajectory');
    axis equal; grid on;
end

% Function to animate the dumbbell motion
function animate_dumbbell(x1, y1, x2, y2, t)
    figure;
    axis equal;
    axis([-1.5 1.5 -1.5 1.5]); % Adjust axis limits
    grid on;
    hold on;
    
    mass1 = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    mass2 = plot(0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    rod = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    
    for i = 1:length(t)
        set(mass1, 'XData', x1(i), 'YData', y1(i));
        set(mass2, 'XData', x2(i), 'YData', y2(i));
        set(rod, 'XData', [x1(i), x2(i)], 'YData', [y1(i), y2(i)]);
        drawnow;
    end
end

% Function defining the system of DAEs for the dumbbell
function zdot = dumbbell_dae(~, z, m, l)
    x1 = z(1); y1 = z(2);
    x2 = z(3); y2 = z(4);
    x1dot = z(5); y1dot = z(6);
    x2dot = z(7); y2dot = z(8);

    % Constraint Equations
    A = [m 0 0 0 -(x2 - x1)/l;  
         0 0 m 0 -(y2 - y1)/l;  
         0 m 0 0 (x2 - x1)/l;
         0 0 0 m (y2 - y1)/l 
         -(x2-x1) (x2-x1) -(y2-y1) (y2-y1) 0];

    B = [0; 0; 0; 0; -((x2dot-x1dot)^2 + (y2dot-y1dot)^2)]; % Constraint equation

    sol = A\B;
    x1ddot = sol(1);
    x2ddot = sol(2);
    y1ddot = sol(3);
    y2ddot = sol(4);    
    
    zdot = [x1dot; y1dot; x2dot; y2dot; x1ddot; y1ddot; x2ddot; y2ddot;];
end
