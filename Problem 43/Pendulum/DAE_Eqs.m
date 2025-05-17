% DAE equations for triple pendulum

% Declaring state and state dot
syms theta1 theta2 theta3 real 
syms thetadot1 thetadot2 thetadot3 real
syms thetaddot1 thetaddot2 thetaddot3 real
syms ROy ROx real

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
    
% Defining distance vectors
rG1 = d1 * er1;

rA = l1 * er1;
rG2_A = d2 * er2;
rG2 = rA + rG2_A;

rB_A = l2 * er2;
rB = rB_A + rA;

rG3_B = d3 * er3;
rG3_A = rB_A + rG3_B;
rG3 = rA + rB_A + rG3_B;

% Accelerations of pivots
a_G1 = simplify(jacobian(jacobian(rG1,z)*zdot,z)*zdot);
a_G2 = simplify(jacobian(jacobian(rG2,z)*zdot,z)*zdot);
a_G3 = simplify(jacobian(jacobian(rG3,z)*zdot,z)*zdot);

a_A = simplify(jacobian(jacobian(rA,z)*zdot,z)*zdot);
a_B = simplify(jacobian(jacobian(rB,z)*zdot,z)*zdot);

% Moments of inertia using parallel axis theorem
I_O1 = I1 + m1*dot(rG1, rG1);
I_O2 = I2 + m2*dot(rG2, rG2);
I_O3 = I3 + m3*dot(rG3, rG3);

I_A2 = I2 + m2*dot(rG2_A, rG2_A);
I_A3 = I3 + m3*dot(rG3_A, rG3_A);

I_B3 = I3 + m3*dot(rG3_B, rG3_B);


% LMB from FBD (I)
LHS = ROy*j + (ROx + m1*g +m2*g + m3*g)*i;
RHS = m1*a_G1 + m2*a_G2 + m3*a_G3;
LMB = LHS - RHS;

eq1 = simplify(dot(LMB, i));
eq2 = simplify(dot(LMB, j));


% AMB about O from FBD (I) 
M_O = dot(cross(rG1, m1*g*i) + cross(rG2, m2*g*i) + cross(rG3, m3*g*i),k);
Hdot_O = I_O1*thetaddot1 + I_O2*thetaddot2 + I_O3*thetaddot3 ...
         + dot(cross(rG1, m1*a_G1) + cross(rG2, m2*a_G2) + cross(rG3, m3*a_G3), k);
eq3 = simplify(M_O - Hdot_O);

% AMB about A from FBD (II)
M_A = dot(cross(rG2_A, m2*g*i) + cross(rG3_A, m3*g*i),k);
Hdot_A = I_A2*thetaddot2 + I_A3*thetaddot3 ...
         + dot(cross(rG2_A, m2*(a_G2 - a_A)) + cross(rG3_A, m3*(a_G3 - a_A)), k);
eq4 = simplify(M_A - Hdot_A);

% AMB about B from FBD (III)
M_B = dot(cross(rG3_B, m3*g*i), k);
Hdot_B = I_B3*thetaddot3 ...
         + dot(cross(rG3_B, m3*(a_G3 - a_B)), k);
eq5 = simplify(M_B - Hdot_B);

% Solve DAE equations for angular accelerations and Constraint forces 
[A, B] = equationsToMatrix([eq1, eq2, eq3, eq4, eq5], [zdot(4:6); ROx; ROy]);
matlabFunction(A, B, 'File', 'DAE_matrices', 'Vars', {z, p});


