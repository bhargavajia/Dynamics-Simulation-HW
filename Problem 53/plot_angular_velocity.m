function plot_angular_velocity(t, omega)

% Plots the angular velocity components over time.
figure;
plot(t, omega(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '\omega_x');
hold on;
plot(t, omega(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', '\omega_y');
plot(t, omega(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '\omega_z');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Components');
legend show;
grid on;

% Optional: Plot the norm of angular velocity
figure;
plot(t, vecnorm(omega, 2, 1), 'k-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Magnitude of Angular Velocity (rad/s)');
title('Magnitude of Angular Velocity');
grid on;

end