function animate_euler(sol_E, tspan, anim_length)

    figure; hold on; axis equal;
    axis([-5 5 -5 5 -5 5]); xlabel('X'); ylabel('Y'); zlabel('Z'); view(3); title('Rotation in 3D (Euler Angle Animation)');
    
    [V, F] = create_geometry();

    num_frames = 300; t_anim = linspace(tspan(1), tspan(end), num_frames);
    t_start_real = tic; t_start_sim = t_anim(1);

    for i = 1:length(t_anim)
        t_current_sim = t_anim(i);
        z_now = deval(sol_E, t_current_sim);
        phi=z_now(1); theta=z_now(2); psi=z_now(3);

        % ZYX Euler angles to Rotation Matrix
        Rx = [ 1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi) ];
        Ry = [ cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta) ];
        Rz = [ cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1 ];

        R_now = Rx * Ry * Rz;
        V_now = (R_now * V')'; cla;

        patch('Vertices',V_now,'Faces',F,'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','k');

        % Plot body axes
        quiver3(0,0,0,R_now(1,1),R_now(2,1),R_now(3,1),'r','LineWidth',2);
        quiver3(0,0,0,R_now(1,2),R_now(2,2),R_now(3,2),'g','LineWidth',2);
        quiver3(0,0,0,R_now(1,3),R_now(2,3),R_now(3,3),'b','LineWidth',2);
        drawnow;

        t_elapsed_real = toc(t_start_real); t_elapsed_sim = t_current_sim - t_anim(1);
        desired_real_time = (t_elapsed_sim / (tspan(end) - tspan(1))) * anim_length;
        pause_time = desired_real_time - t_elapsed_real;
        if pause_time > 0; pause(pause_time); end
    end
end

function [V, F] = create_geometry()

    % Define box vertices (Small X, medium Y, long Z)
    Lx=0.2; Ly=1.0; Lz=2.0;
    V = 0.5 * [ -Lx -Ly -Lz; Lx -Ly -Lz; Lx  Ly -Lz; -Lx  Ly -Lz;
                -Lx -Ly  Lz; Lx -Ly  Lz; Lx  Ly  Lz; -Lx  Ly  Lz];

    % Define faces
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end