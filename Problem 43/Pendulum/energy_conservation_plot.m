function energy_conservation_plot(sol, p)
    % Parameters
    g = 10;  % gravity
    
    % Time vector and fine interpolation for accuracy
    t_fine = linspace(sol.x(1), sol.x(end), 500);
    z = deval(sol, t_fine);  % z = [theta1; theta2; theta3; thetadot1; thetadot2; thetadot3]

    % Extract states
    theta1 = z(1, :);
    theta2 = z(2, :);
    theta3 = z(3, :);
    thetadot1 = z(4, :);
    thetadot2 = z(5, :);
    thetadot3 = z(6, :);

    % Unit vectors (in original frame)
    i = [1; 0];
    j = [0; 1];
    
    % Compute velocities of centers of mass (scalar since 2D)
    % Using cross(k, w*r) => here k is out of plane, w = thetadot
    % Cross product magnitude = w * r perpendicular to er (unit vector along link)
    
    % For each link: position of COM relative to origin
    % Er vectors for each link
    er1 = [cos(theta1); sin(theta1)];
    er2 = [cos(theta2); sin(theta2)];
    er3 = [cos(theta3); sin(theta3)];

    % Velocities of COM
    v1x = -thetadot1 .* p.d1 .* sin(theta1);
    v1y =  thetadot1 .* p.d1 .* cos(theta1);
    
    v2x = -thetadot1 .* p.l1 .* sin(theta1) - thetadot2 .* p.d2 .* sin(theta2);
    v2y =  thetadot1 .* p.l1 .* cos(theta1) + thetadot2 .* p.d2 .* cos(theta2);
    
    v3x = -thetadot1 .* p.l1 .* sin(theta1) - thetadot2 .* p.l2 .* sin(theta2) - thetadot3 .* p.d3 .* sin(theta3);
    v3y =  thetadot1 .* p.l1 .* cos(theta1) + thetadot2 .* p.l2 .* cos(theta2) + thetadot3 .* p.d3 .* cos(theta3);
    
    % Speeds squared
    v1_sq = v1x.^2 + v1y.^2;
    v2_sq = v2x.^2 + v2y.^2;
    v3_sq = v3x.^2 + v3y.^2;
    
    % Kinetic energy
    Ek = 0.5 * (p.m1 * v1_sq + p.m2 * v2_sq + p.m3 * v3_sq + ...
                p.I1 * thetadot1.^2 + p.I2 * thetadot2.^2 + p.I3 * thetadot3.^2);
            
    % Potential energy (height measured along vertical direction, your x axis)
    % Heights of COM from original code
    h1 = p.d1 * cos(theta1);
    h2 = p.l1 * cos(theta1) + p.d2 * cos(theta2);
    h3 = p.l1 * cos(theta1) + p.l2 * cos(theta2) + p.d3 * cos(theta3);
    
    Ep = - g * (p.m1 * h1 + p.m2 * h2 + p.m3 * h3);

    % Sum of Energies 
    dE = Ek + Ep;
    
    % Plot
    figure;
    plot(t_fine, dE, 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('E_k + E_p');
    title('Sum of Kinetic and Potential Energy');
    grid on;
end
