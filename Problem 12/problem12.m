tend = 10;
tspan = [0,tend]; % Simulation length

% Initial conditions
r0 = [0;0];
theta = pi/4;
v0_mag = 1;
v0 = [v0_mag*cos(theta); v0_mag*sin(theta)];
z0 = [r0;v0];
m = 1;

%Modify the z0 vector to include work as an intial value
z0_new = [z0; 0];

%Modify myrhs so that it can take the third equation
function zdot = myrhs(t,z)
    r = z(1:2);
    v = z(3:4);
    W = z(5);

    i = [1; 0]; j = [0; 1];
    m = 1;
    c = 1;
    g = 1;
    
    Fg = - m * g * j;
    Fdrag = - c * v * norm(v);
    Ftot = Fg + Fdrag;
    
    rdot = v;
    vdot = Ftot/m;

    %Calculate power
    P = dot(Fdrag,v);
    dWdt = P;
    zdot = [rdot; vdot; dWdt];
end

% Define different tolerance values
tolerances = [1e-2, 1e-4, 1e-6, 1e-8, 1e-10]; %Different tolerances for accuracies
num_tolerances = length(tolerances);

EnergyDifferences = zeros(1, num_tolerances); % stores the energy differences

% Loop through different tolerances
for i = 1:num_tolerances
    % Solve the system of ODEs with the current tolerance
    options = odeset('RelTol', tolerances(i), 'AbsTol', tolerances(i));
    solution = ode45(@myrhs, tspan, z0_new, options);
    
    % Extract the work variable from the output
    t = linspace(0, tend, 100);
    z = deval(solution, t);
    x = z(1, :); % Extract x values
    y = z(2, :); % Extract y values
    W = z(5,:); % Extract work value
    
    % Calculate the change in total energy
    KE_initial = 0.5 * m * v0_mag^2;
    PE_initial = m * 1 * r0(2); % set g = 1
    E_initial = KE_initial + PE_initial;
    
    KE_final = 0.5 * m * (z(3,:).^2 + z(4,:).^2);
    PE_final = m * 1 .* y;
    E_final = KE_final + PE_final;
    
    deltaE = E_final - E_initial;
    
    % Compare work done by drag force to the change in energy
    EnergyDifference = W(end) - deltaE(end); % Take the final values
    EnergyDifferences(i) = abs(EnergyDifference); %Store the absolute difference
end

% Plot the solution trajectory
figure;
plot(x,y,'LineWidth',2)
hold on;
axis equal
grid on
xlabel("X axis")
ylabel("Y axis")
title("Trajectory of cannonball with drag")

% Plot the energy difference vs tolerances
figure;
loglog(tolerances, EnergyDifferences,'-o','LineWidth',2);
xlabel("Tolerance (Integration Accuracy)");
ylabel("Absolute Difference between Work and Change in Energy");
title("Convergence of Energy Difference with Integration Accuracy");
grid on
