clear; clc; close all;

% Inertia along each axis
I = [0.3, 0.6, 1.0];

% Initial conditions for angular velocity 

% w0 = [0.5; 0.3; 2.0]; % for part (i)

% w0 = [1; 0.01; 0.01]; % mostly about x axis
% w0 = [0.01; 0.01; 1]; % mostly about z axis

w0 = [0.01; 1; 0.01]; % for part (ii) - mostly about y axis


% Initialising R as identity
R0 = eye(3);

% Time span
tspan = [0 20];

% Combine state vector: [w; R_flattened]
z0 = [w0; R0(:)];

% Solve the system using ode45 
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
sol = ode45(@(t, z) combined_system(t, z, I), tspan, z0, options);

% Animate with time scaling
animate(sol, tspan, 10);  % 10 seconds real time animation

% Plot angular velocity
t = sol.x;
z = sol.y;
omega = z(1:3, :);
plot_angular_velocity(t, omega);


function zdot = combined_system(t, z, I)
    w = z(1:3);
    R_flat = z(4:end);
    R = reshape(R_flat, 3, 3);

    % wdot
    wdot = wdot_eq(t, w, I);

    % Rdot
    Rdot = Rdot_eq(t, R, w);

    % Combine
    zdot = [wdot; Rdot(:)];
end




