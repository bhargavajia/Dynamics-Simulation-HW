function animate(sol, p)
    % Extract time and state
    tspan = sol.x;
    y = sol.y;
    
    % Fine interpolation for smooth animation
    t_fine = linspace(tspan(1), tspan(end), 1000);
    y_fine = deval(sol, t_fine);
    
    % Desired animation duration (in seconds)
    anim_duration = 5;
    
    % Extract angles
    theta1 = y_fine(1, :);
    theta2 = y_fine(2, :);
    theta3 = y_fine(3, :);
    
    % Positions of joints (in original frame)
    x1 = p.l1 * cos(theta1);
    y1 = p.l1 * sin(theta1);
    
    x2 = x1 + p.l2 * cos(theta2);
    y2 = y1 + p.l2 * sin(theta2);
    
    x3 = x2 + p.l3 * cos(theta3);
    y3 = y2 + p.l3 * sin(theta3);
    
    % Positions of centers of mass (in original frame)
    xc1 = p.d1 * cos(theta1);
    yc1 = p.d1 * sin(theta1);
    
    xc2 = x1 + p.d2 * cos(theta2);
    yc2 = y1 + p.d2 * sin(theta2);
    
    xc3 = x2 + p.d3 * cos(theta3);
    yc3 = y2 + p.d3 * sin(theta3);
    
    % Flip frame: rotate 90 deg so pendulum is vertical in plot
    x1_plot = y1;
    y1_plot = -x1;
    
    x2_plot = y2;
    y2_plot = -x2;
    
    x3_plot = y3;
    y3_plot = -x3;
    
    xc1_plot = yc1;
    yc1_plot = -xc1;
    
    xc2_plot = yc2;
    yc2_plot = -xc2;
    
    xc3_plot = yc3;
    yc3_plot = -xc3;
    
    % Total length for axis limits
    Ltotal = p.l1 + p.l2 + p.l3;
    
    % Fixed axis limits with more space at bottom
    xlim_plot = [-Ltotal, Ltotal];
    ylim_plot = [-(Ltotal + 0.5*Ltotal), 0.5*Ltotal];
    
    % Initialize figure
    figure;
    axis equal;
    axis([xlim_plot, ylim_plot]);
    hold on;
    grid on;
    xlabel('Y');
    ylabel('-X');
    title('Triple Pendulum Animation');
    
    % Plot fixed horizontal bar at top
    bar_width = 0.5 * Ltotal;
    plot([-bar_width/2, bar_width/2], [0, 0], 'k-', 'LineWidth', 2, 'DisplayName', 'Fixed Bar');
    
    % Initialize pendulum links
    line1 = plot([0, 0], [0, 0], 'r-', 'LineWidth', 2, 'DisplayName', 'Link 1');
    line2 = plot([0, 0], [0, 0], 'g-', 'LineWidth', 2, 'DisplayName', 'Link 2');
    line3 = plot([0, 0], [0, 0], 'b-', 'LineWidth', 2, 'DisplayName', 'Link 3');
    
    % Centers of mass markers
    com1 = plot(0, 0, 'rx', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'CoM 1');
    com2 = plot(0, 0, 'gx', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'CoM 2');
    com3 = plot(0, 0, 'bx', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'CoM 3');
    
    legend('Location', 'northeastoutside');
    
    % Frame time to match anim_duration
    frame_dt = anim_duration / length(t_fine);
    
    % Start animation timing
    t_start = tic;
    
    % Animation loop with corrected timing
    for k = 1:length(t_fine)
        % Update links
        set(line1, 'XData', [0, x1_plot(k)], 'YData', [0, y1_plot(k)]);
        set(line2, 'XData', [x1_plot(k), x2_plot(k)], 'YData', [y1_plot(k), y2_plot(k)]);
        set(line3, 'XData', [x2_plot(k), x3_plot(k)], 'YData', [y2_plot(k), y3_plot(k)]);
        
        % Update centers of mass
        set(com1, 'XData', xc1_plot(k), 'YData', yc1_plot(k));
        set(com2, 'XData', xc2_plot(k), 'YData', yc2_plot(k));
        set(com3, 'XData', xc3_plot(k), 'YData', yc3_plot(k));
        
        drawnow;
        
        % Absolute time correction
        elapsed = toc(t_start);
        target_time = (k-1) * frame_dt;
        pause_time = target_time - elapsed;
        if pause_time > 0
            pause(pause_time);
        end
    end
end
