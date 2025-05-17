function generateDAE()

    % Declare symbolic variables
    syms x y theta xdot ydot thetadot real
    syms xddot yddot thetaddot real 
    syms F1 F2 theta1 theta2 real
    syms m I d1 d2 real
    z = [x;y;theta;xdot;ydot;thetadot];
    p = [m; I; d1; d2; theta1; theta2];
    unknowns = [xddot; yddot; thetaddot; F1; F2];

    % Basis vectors
    i = [1;0;0];
    j = [0;1;0];
    k = cross(i,j);
    f1 = -sin(theta+theta1)*i + cos(theta+theta1)*j;
    f2 = -sin(theta+theta2)*i + cos(theta+theta2)*j;
    lambda = cos(theta)*i + sin(theta)*j;
    
    % Skate DAE equations
    % LMB
    eq1 = dot((F1*f1 + F2*f2), i) - m*xddot;
    eq2 = dot((F1*f1 + F2*f2), j) - m*yddot;
    
    % AMB
    M_G = cross(-d1*lambda, f1*F1) + cross(d2*lambda, f2*F2);
    eq3 = simplify(dot(M_G, k) - I*thetaddot);
    
    % Constraint equations
    zdot = [xdot; ydot; thetadot; xddot; yddot; thetaddot];
    
    % Skate 1
    v1 = cross(thetadot*k, -d1*lambda) + xdot*i + ydot*j;
    c1 = dot(v1,f1);
    eq4 = simplify(jacobian(c1,z) * zdot);
    
    % Skate 2
    v2 = cross(thetadot*k, d2*lambda) + xdot*i + ydot*j;
    c2 = dot(v2,f2);
    eq5 = simplify(jacobian(c2,z) * zdot);
    
    % Solve DAE for xddot, yddot, thetaddot, F1, F2
    eqs = [eq1; eq2; eq3; eq4; eq5];
    [A, B] = equationsToMatrix(eqs, unknowns);

    matlabFunction(A, B, 'File', 'DAE_matrices', 'Vars', {z, p});
end
