% Central Force Motion Exploration      

% Force function
function F = central_force(r, force_type, k, coeffs)
  % r: distance from the center
  % force_type: 'inverse_power', 'linear', 'polynomial'.
  % k: force constant
  % coeffs: coefficients for polynomial force

  switch force_type
    case 'inverse_power'
      F = -k ./ (r.^coeffs(1)); % F = -k/r^alpha, coeffs(1) is alpha
    case 'linear'
      F = -k .* r;        % F = -kr
    case 'polynomial'
      %F = c0 + c1*r + c2*r^2 + ...
      F = 0;
      for i = 1:length(coeffs)
        F = F + coeffs(i) * r.^(i-1);
      end
    case 'new'
       F = -k ./ r - k.*r - k*r.^2;
    otherwise
      error('Invalid force type');
  end
end

% Define the equations of motion
function dy = force_eqns(t, y, force_type, k, coeffs, m)
  % y = [r, theta, r_dot, theta_dot]
  r = y(1);
  theta = y(2);
  r_dot = y(3);
  theta_dot = y(4);

  % Calculate force
  F = central_force(r, force_type, k, coeffs);

  % Equations of motion in polar coordinates
  r_ddot = r * theta_dot^2 + F/m;
  theta_ddot = -2 * r_dot * theta_dot / r;

  dy = [r_dot; theta_dot; r_ddot; theta_ddot];
end

% Main script
m = 1;
force_type = 'new';  
k = 1;                      
coeffs = [1,1,1,1]; 
T = 200;                  
r0 = 10;                  
r_dot0 = 0; 
theta0 = 0;     
theta_dot0 = 1;   
y0 = [r0, theta0, r_dot0, theta_dot0];

% Solve the equations of motion with arbitrary initial conditions.
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
[t,y] = ode45(@(t,y) force_eqns(t, y, force_type, k, coeffs,m), [0, T], y0, options);

% Plot the trajectory
figure;
polarplot(y(:,2), y(:,1));
title(['Trajectory for ' force_type ' Force']);
hold on;
polarplot(y(1,2), y(1,1), 'ro'); %Mark the initial position
hold off;


