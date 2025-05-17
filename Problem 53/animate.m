function animate(sol, tspan, anim_length)
    figure;
    hold on;
    axis equal;
    axis([-2 2 -2 2 -0.5 0.5]*2);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);

    [V, F] = create_geometry();

    t_anim = linspace(tspan(1), tspan(end), 500);
    t_start_real = tic;
    t_start_sim = t_anim(1);

    for i = 1:length(t_anim)
        t_current_sim = t_anim(i);
        z_now = deval(sol, t_current_sim);
        R_now = reshape(z_now(4:end), 3, 3);
        V_now = (R_now * V')';

        cla;
        patch('Vertices',V_now,'Faces',F,'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k');
        quiver3(0,0,0,R_now(1,1),R_now(2,1),R_now(3,1),'r','LineWidth',2);
        quiver3(0,0,0,R_now(1,2),R_now(2,2),R_now(3,2),'g','LineWidth',2);
        quiver3(0,0,0,R_now(1,3),R_now(2,3),R_now(3,3),'b','LineWidth',2);
        title('Rotation in 3D');
        drawnow;

        t_elapsed_real = toc(t_start_real);
        t_elapsed_sim = t_current_sim - t_start_sim;
        desired_real_time = (t_elapsed_sim / (tspan(end) - tspan(1))) * anim_length;
        pause_time = desired_real_time - t_elapsed_real;
        if pause_time > 0
            pause(pause_time);
        end
    end
end

function [V, F] = create_geometry()
    % Small in X, medium in Y, long in Z
    Lx = 0.2;
    Ly = 1.0;
    Lz = 2.0;
    
    V = 0.5 * [ -Lx -Ly -Lz;
                 Lx -Ly -Lz;
                 Lx  Ly -Lz;
                -Lx  Ly -Lz;
                -Lx -Ly  Lz;
                 Lx -Ly  Lz;
                 Lx  Ly  Lz;
                -Lx  Ly  Lz];
    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];
end
