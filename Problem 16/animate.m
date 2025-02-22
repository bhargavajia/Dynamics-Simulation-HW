function retval = animate(solution, tspan, z0, time_scale)
    % Unpacking values to calculate limits %
    z_vals = deval(solution, linspace(tspan(1), tspan(2), 500));
    r1_vals = z_vals(1:2,:);
    r2_vals = z_vals(3:4,:);

    % Figure Setup %
    figure; hold on;
    grid on; axis equal;

    xlim([min([r1_vals(1,:), r2_vals(1,:)]) - 0.5, max([r1_vals(1,:), r2_vals(1,:)]) + 0.5]);
    ylim([min([r1_vals(2,:), r2_vals(2,:)]) - 0.5, max([r1_vals(2,:), r2_vals(2,:)]) + 0.5]);


    xlabel("x-position (m)");
    ylabel("y-position (m)");
    title("Animation of 2 particles", "FontSize",20)

    plot(r1_vals(1,:),r1_vals(2,:), 'LineStyle','-', 'Color', 'r');
    plot(r2_vals(1,:),r2_vals(2,:), 'LineStyle','-.', 'Color','b')
    m1 = plot(z0(1), z0(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    m2 = plot(z0(3), z0(4), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

    legend('Mass 1', 'Mass 2');

    total_time = tspan(2) - tspan(1);
    t_start = tic();

    while true
        elapsed_time = toc(t_start) * time_scale;

        if elapsed_time > total_time
            break;
        end

        z_current = deval(solution, elapsed_time);
        set(m1, 'XData', z_current(1), 'YData', z_current(2));
        set(m2, 'XData', z_current(3), 'YData', z_current(4));

        drawnow;
    end
end
