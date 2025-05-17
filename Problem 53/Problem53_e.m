clear; clc; close all;

% Euler Angles Solution

% Inertia vector along each axis 
I_vec = [0.3, 0.6, 1.0]; 

% Initial conditions for angular velocity (mostly about y axis)
w0 = [0.001; 1.0; 0.001];

% Initial conditions for Euler angles (ZYX convention)
% For initial rotation matrix identity
phi0 = 0;   % Rotation about body x-axis
theta0 = 0; % Rotation about body y'-axis
psi0 = 0;   % Rotation about body z''-axis

% Time span
tspan = [0 20];

% Combine initial state vector for Euler Angle approach:
z0_E = [phi0; theta0; psi0; w0];

% Solve the system using ode45 (Euler Angle approach)
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
sol_E = ode45(@(t, z) combined_system_E(t, z, I_vec), tspan, z0_E, options); % Pass the inertia vector


% Animate Euler angle solution
animate_euler(sol_E, tspan, 10)


% Extract and Plot Results from Euler Angle Solution
t_E = sol_E.x;
z_E = sol_E.y;

% Extract Euler angles
phi_E = z_E(1,:);
theta_E = z_E(2,:);
psi_E = z_E(3,:);

% Extract angular velocity components
omega_E = z_E(4:6, :);

% Plot angular velocity components
figure;
plot(t_E, omega_E(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '\omega_x');
hold on;
plot(t_E, omega_E(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', '\omega_y');
plot(t_E, omega_E(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '\omega_z');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Components (Euler Angle Method)');
legend show;
grid on;

% Plot the norm of angular velocity
figure;
plot(t_E, vecnorm(omega_E, 2, 1), 'k-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Magnitude of Angular Velocity (rad/s)');
title('Magnitude of Angular Velocity (Euler Angle Method)');
grid on;


% Function definition for the Euler Angle system
function zdot = combined_system_E(t, z, I_vec)

    phi = z(1);
    theta = z(2);
    psi = z(3);
    omega = z(4:6); % Angular velocity in body frame [wx; wy; wz]

    cos_theta = cos(theta);
    sin_phi = sin(phi);
    cos_phi = cos(phi);

    if abs(cos_theta) < 1e-6 && abs(theta) > 1e-6
        error('Gimbal lock condition approached (theta near +/- pi/2). The Euler angle representation is singular here.');
    end

    % Calculate the Euler angle rates using the transformation

    if abs(cos_theta) < 1e-10 && abs(theta) > 1e-6 % If theta is near +/- pi/2 and not zero
         error('Gimbal lock condition encountered unexpectedly during calculation.');
    else
        tan_theta = tan(theta);
    end

    phi_dot   = omega(1) + omega(2) * sin_phi * tan_theta + omega(3) * cos_phi * tan_theta;
    theta_dot = omega(2) * cos_phi - omega(3) * sin_phi;
    psi_dot   = (omega(2) * sin_phi + omega(3) * cos_phi) / cos_theta;


    omegaddot = wdot_eq(t, omega, I_vec); % Call the user's wdot_eq function

    % Combine the derivatives into the state vector derivative zdot
    % zdot = [phi_dot; theta_dot; psi_dot; omega_dot_x; omega_dot_y; omega_dot_z]
    zdot = [phi_dot; theta_dot; psi_dot; omegaddot(1); omegaddot(2); omegaddot(3)];
end
