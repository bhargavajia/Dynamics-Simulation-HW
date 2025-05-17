function animate(sol, p)
    % Animate Two Skates Simulation with perpendicular dotted lines to skates

    % Fine time and state interpolation
    t_fine = linspace(sol.x(1), sol.x(end), 500);
    z_fine = deval(sol, t_fine);
    timescale = (sol.x(end) - sol.x(1)) / 10;  % For a 10 sec simulation

    % Parameters from p
    d1 = p.d1;
    d2 = p.d2;
    theta1 = p.theta1;
    theta2 = p.theta2;

    % Ellipse parameters (large enough to fit skates)
    a = 0.4; % along lambda
    b = 0.15; % perpendicular to lambda

    % Skate line length (centered at skate position)
    skate_line_length = 0.2;

    % Dotted line extension (now perpendicular to each skate direction)
    line_extension = 5;

    % Calculate overall axis limits based on trajectory
    x_min = min(z_fine(1,:)) - 2;
    x_max = max(z_fine(1,:)) + 2;
    y_min = min(z_fine(2,:)) - 2;
    y_max = max(z_fine(2,:)) + 2;

    % Prepare figure
    figure;
    axis equal
    grid on
    hold on
    xlabel('X')
    ylabel('Y')
    title('Two Skates Animation')

    % Precompute ellipse shape
    t_ellipse = linspace(0, 2*pi, 100);
    unit_ellipse = [a*cos(t_ellipse); b*sin(t_ellipse)];

    for i = 1:length(t_fine)
        cla;

        % State
        x = z_fine(1,i);
        y = z_fine(2,i);
        theta = z_fine(3,i);

        % Lambda and perpendicular
        lambda = [cos(theta); sin(theta)];
        lambda_perp = [-sin(theta); cos(theta)];

        % Skate positions
        skate1_pos = [x; y] - d1 * lambda;
        skate2_pos = [x; y] + d2 * lambda;

        % Skate directions
        skate1_dir = rotation2D(theta1) * lambda;
        skate2_dir = rotation2D(theta2) * lambda;

        % Perpendicular directions to skates
        skate1_perp = [-skate1_dir(2); skate1_dir(1)];
        skate2_perp = [-skate2_dir(2); skate2_dir(1)];

        % Dotted lines perpendicular to skate1_dir from skate1_pos
        plot([skate1_pos(1) - line_extension * skate1_perp(1), skate1_pos(1) + line_extension * skate1_perp(1)], ...
             [skate1_pos(2) - line_extension * skate1_perp(2), skate1_pos(2) + line_extension * skate1_perp(2)], 'r:', 'LineWidth', 1);

        % Dotted lines perpendicular to skate2_dir from skate2_pos
        plot([skate2_pos(1) - line_extension * skate2_perp(1), skate2_pos(1) + line_extension * skate2_perp(1)], ...
             [skate2_pos(2) - line_extension * skate2_perp(2), skate2_pos(2) + line_extension * skate2_perp(2)], 'm:', 'LineWidth', 1);

        % Lambda line only inside ellipse
        plot([x - a*lambda(1), x + a*lambda(1)], [y - a*lambda(2), y + a*lambda(2)], 'b-', 'LineWidth', 1);

        % Skate1 line (centered at skate1_pos, symmetric along skate1_dir)
        skate1_start = skate1_pos - 0.5 * skate_line_length * skate1_dir;
        skate1_end   = skate1_pos + 0.5 * skate_line_length * skate1_dir;
        plot([skate1_start(1), skate1_end(1)], [skate1_start(2), skate1_end(2)], 'r', 'LineWidth', 2);

        % Skate2 line (centered at skate2_pos, symmetric along skate2_dir)
        skate2_start = skate2_pos - 0.5 * skate_line_length * skate2_dir;
        skate2_end   = skate2_pos + 0.5 * skate_line_length * skate2_dir;
        plot([skate2_start(1), skate2_end(1)], [skate2_start(2), skate2_end(2)], 'm', 'LineWidth', 2);

        % Body ellipse
        R = [lambda, lambda_perp];
        ellipse_points = R * unit_ellipse + [x; y];
        fill(ellipse_points(1,:), ellipse_points(2,:), 'c', 'FaceAlpha', 0.3, 'EdgeColor', 'b');

        % Body center
        plot(x, y, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

        % Dynamic axis
        xlim([x_min, x_max]);
        ylim([y_min, y_max]);

        drawnow;
        pause((t_fine(2)-t_fine(1)) / timescale);
    end
end

function R = rotation2D(angle)
    R = [cos(angle), -sin(angle);
         sin(angle),  cos(angle)];
end
