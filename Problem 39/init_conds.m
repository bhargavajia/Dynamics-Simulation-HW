
function [xdot, ydot] = init_conds(thetadot, theta, theta1, theta2, d1, d2)
    syms xdot ydot real

    i = [1;0;0];
    j = [0;1;0];
    k = cross(i,j);
    f1 = -sin(theta+theta1)*i + cos(theta+theta1)*j;
    f2 = -sin(theta+theta2)*i + cos(theta+theta2)*j;
    lambda = cos(theta)*i + sin(theta)*j;

    % Skate 1
    v1 = cross(thetadot*k, -d1*lambda) + xdot*i + ydot*j;
    c1 = dot(v1,f1);
    
    % Skate 2
    v2 = cross(thetadot*k, d2*lambda) + xdot*i + ydot*j;
    c2 = dot(v2,f2);

    [xdot, ydot] = solve([c1,c2],[xdot, ydot]);
    xdot = double(xdot);
    ydot = double(ydot);
end
