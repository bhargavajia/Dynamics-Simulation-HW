animate_house()

function animate_house()
    % Parameters
    tspan = [0, 100]; % Time span for the animation
    time_scale = 5;  % Speed of the animation
    R = 5;           % Radius of the circular path
    omega = 4 * pi / tspan(2); % Angular speed
    rotation_speed = 50; % Rotation speed in degrees per second

    % Define the "house" as a polygon
    house_x = [-0.5, 0.5, 0.5, -0.5, -0.5]; % House base and roof
    house_y = [0, 0, 0.5, 0.5, 0];
    roof_x = [-0.6, 0, 0.6, -0.6];
    roof_y = [0.5, 0.8, 0.5, 0.5];
    house_color = 'y'; % House color
    roof_color = 'r'; % Roof color

    % Initialize the figure
    figure;
    hold on;
    axis equal;
    axis([-6, 6, -6, 6]); % Set fixed axis limits
    grid off;
    shg;

    % Initialize plot object
    house = fill(house_x, house_y, house_color, 'EdgeColor', 'k', 'LineWidth', 1.5);
    roof = fill(roof_x, roof_y, roof_color, 'EdgeColor', 'k', 'LineWidth', 1.5);

    % Total animation duration
    total_time = tspan(2) - tspan(1);

    % Start animation loop
    t_start = tic; % Start timer
    while true
        % Elapsed time since animation started
        elapsed_time = toc(t_start) * time_scale;

        % Check if animation is complete
        if elapsed_time >= total_time
            break;
        end

        % Compute current position on the circular path
        theta = omega * elapsed_time;
        x_center = R * cos(theta);
        y_center = R * sin(theta);

        % Compute rotation angle
        rotation_angle = rotation_speed * elapsed_time;

        % Apply rotation and translation to the house
        [rotated_x_house, rotated_y_house] = rotate_shape(house_x, house_y, rotation_angle);
        translated_x_house = rotated_x_house + x_center;
        translated_y_house = rotated_y_house + y_center;
        set(house, 'XData', translated_x_house, 'YData', translated_y_house);

        % Apply rotation and translation to the roof
        [rotated_x_roof, rotated_y_roof] = rotate_shape(roof_x, roof_y, rotation_angle);
        translated_x_roof = rotated_x_roof + x_center;
        translated_y_roof = rotated_y_roof + y_center;
        set(roof, 'XData', translated_x_roof, 'YData', translated_y_roof);

        % Pause to create animation effect
        drawnow;
    end

    hold off;
end

function [rotated_x, rotated_y] = rotate_shape(x, y, angle)
    % Rotate a 2D shape around its center
    angle_rad = deg2rad(angle); % Convert angle to radians
    rotation_matrix = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
    rotated_coords = rotation_matrix * [x; y];
    rotated_x = rotated_coords(1, :);
    rotated_y = rotated_coords(2, :);
end


