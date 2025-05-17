% Lagrange equations for triple pendulum

% Declaring state and state dot
syms theta1 theta2 theta3 real 
syms thetadot1 thetadot2 thetadot3 real
syms thetaddot1 thetaddot2 thetaddot3 real

z = [theta1 theta2 theta3 thetadot1 thetadot2 thetadot3]';
zdot = [thetadot1 thetadot2 thetadot3 thetaddot1 thetaddot2 thetaddot3]';

% Declaring lengths, masses and Inertia
syms l1 l2 l3 d1 d2 d3 real
syms I1 I2 I3 m1 m2 m3 real

p = [l1 l2 l3 d1 d2 d3 I1 I2 I3 m1 m2 m3]';

% Constants 
g = 10;

% Unit vectors 
i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

er1 = cos(theta1)*i + sin(theta1)*j;
etheta1 = cross(k, er1);

er2 = cos(theta2)*i + sin(theta2)*j;
etheta2 = cross(k, er2);

er3 = cos(theta3)*i + sin(theta3)*j;
etheta3 = cross(k, er3);
    

% Velocities of center of masses of each link
v1 = cross(thetadot1*k, d1*er1);
v2 = cross(thetadot1*k, l1*er1) + cross(thetadot2*k, d2*er2);
v3 = cross(thetadot1*k, l1*er1) + cross(thetadot2*k, l2*er2) + cross(thetadot3*k, d3*er3);

% Heights of center of masses of each link
h1 = d1 * cos(theta1);
h2 = l1 * cos(theta1) + d2 * cos(theta2);
h3 = l1 * cos(theta1) + l2 * cos(theta2) + d3 * cos(theta3);

% Kinetic Energy
Ek = 1/2 * (m1*dot(v1, v1) + m2*dot(v2, v2) + m3*dot(v3, v3) ...
            + I1*thetadot1^2 + I2*thetadot2^2 + I3*thetadot3^2);

% Potential energy
Ep = - g * (m1*h1 + m2*h2 + m3*h3);

% Lagrange equations
q = z(1:3);
qdot = zdot(1:3);
L = Ek - Ep;
L_q = gradient(L, q);
L_qdot = gradient(L, qdot);
L_qdot_t = jacobian(L_qdot, z) * zdot;

L_eqs = simplify(L_qdot_t - L_q);

% Solve Lagrange equations for angular accelerations
[A, B] = equationsToMatrix(L_eqs, zdot(4:6));

matlabFunction(A, B, 'File', 'Lagrange_matrices', 'Vars', {z, p});
