function generateLagrange()
    % Declare symbolic variables
    syms x y theta xdot ydot thetadot real
    syms xddot yddot thetaddot real 
    syms F1 F2 theta1 theta2 real
    syms m I d1 d2 real
    z = [x;y;theta;xdot;ydot;thetadot];
    zdot = [xdot; ydot; thetadot; xddot; yddot; thetaddot];
    p = [m; I; d1; d2; theta1; theta2];
    unknowns = [xddot; yddot; thetaddot; F1; F2];

    % Basis vectors
    i = [1;0;0];
    j = [0;1;0];
    k = cross(i,j);
    f1 = -sin(theta+theta1)*i + cos(theta+theta1)*j;
    f2 = -sin(theta+theta2)*i + cos(theta+theta2)*j;
    lambda = cos(theta)*i + sin(theta)*j;

    % Lagrange equations
    v = xdot*i + ydot*j;
    Ek = 1/2 * (m * dot(v,v)^2 + I*thetadot^2);
    Ep = 0;

    L = Ek - Ep;  % Lagrangian

    q = [x; y; theta];  % Lagrange variable 
    qdot = [xdot; ydot; thetadot];
    qddot = [xddot; yddot; thetaddot];

    L_qdot = gradient(L,qdot);
    L_q = gradient(L,q);
    L_qdot_t = jacobian(L_qdot, z) * zdot;

    Leq = L_qdot_t - L_q;

    % Finding Qi (generalized forces)
    Q_x = dot((F1*f1 + F2*f2), i);  % sum of Fx
    Q_y = dot((F1*f1 + F2*f2), j);  % sum of Fy
    Q_theta = simplify(dot((cross(-d1*lambda, f1*F1) + cross(d2*lambda, f2*F2)), k));
    % Sum of moments 

    Q = [Q_x; Q_y; Q_theta];

    % Combining Qi with lagrange equations
    eqs = Leq - Q;

    % Constraint equations
    % Skate 1
    v1 = cross(thetadot*k, -d1*lambda) + xdot*i + ydot*j;
    c1 = dot(v1,f1);
    eqs(4) = simplify(jacobian(c1,z) * zdot);
    
    % Skate 2
    v2 = cross(thetadot*k, d2*lambda) + xdot*i + ydot*j;
    c2 = dot(v2,f2);
    eqs(5) = simplify(jacobian(c2,z) * zdot);
    
    % Solve Lagrange DAEs for xddot, yddot, thetaddot, F1, F2
    [A, B] = equationsToMatrix(eqs, unknowns);

    matlabFunction(A, B, 'File', 'Lagrange_matrices', 'Vars', {z, p});
end
