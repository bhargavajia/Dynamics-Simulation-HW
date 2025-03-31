clc;
close all;
clear all;

% Spring and mass problem non zero length
exitflag = 1;

% % Parameters and initial conditions
% rA = [3 0]';
% rB = [-3 0]';
% kA = 50; LA = 5; cA = 1;
% kB = 50; LB = 5; cB = 1;
% m = 1;
% g = 0;
% c = 0.2;

% Parameters and initial conditions
rA = [5 0]';
rB = [-5 0]';
kA = 10; LA = 20; cA = 1;
kB = 50; LB = 10; cB = 1;
m = 100;
g = 10;
c = 0;

% Inital guess
r0 = [11; 0.001];
v0 = [0; 0];
z0 = [r0; v0]; 

% STATICS SOLUTION
mystatics = @(z) ...
            sumofforces(z(1:2), [0;0], rA, rB, kA, LA, cA, kB, LB, cB, m, g, c);
options = optimoptions('fsolve', 'FunctionTolerance',1e-30, ...
'OptimalityTolerance',1e-8, 'MaxFunctionEvaluations',10000, 'MaxIterations',10000, 'Disp','off');

[root, fval, exitflag] = fsolve(mystatics,z0(1:2),options);
if exitflag < 1
    disp('JIA says: FSOLVE is not happy. We want fval to be close to zeros.');
    fval
end

disp('The x and y equilibrium point is')
disp(root);


% DYNAMICS
mydynamics = @(z) ...
            sumofforces(z(1:2), z(3:4), rA, rB, kA, LA, cA, kB, LB, cB, m, g, c);

myrhs = @(t,z) [z(3:4); mydynamics(z)/m];
tspan = linspace(0,20,1000);
options = odeset('AbsTol',1e-3,'RelTol',1e-3);
[tarray,zarray] = ode45(myrhs, tspan, z0, options);
x = zarray(:,1); y = zarray(:,2);
figure;
plot(x,y,'*', 'MarkerSize',10); axis equal;
title("Motion of mass")
xlabel("X axis")
ylabel("Y axis")
shg;


% Sum of forces on mass
function F = sumofforces(rC, rCdot, rA, rB, kA, LA, cA, kB, LB, cB, m, g, c);
    i = [1 0]'; j = [0 1]'; 
    rAC = rC - rA; LAC = norm(rAC); lambdaAC = rAC/LAC;
    rBC = rC - rB; LBC = norm(rBC); lambdaBC = rBC/LBC;

    LACdot = dot(rCdot, lambdaAC);
    LBCdot = dot(rCdot, lambdaBC);

    F = - kA * (LAC-LA) * lambdaAC ... % spring a
        - cA * LACdot * lambdaAC ... % dashpot a
        - kB * (LBC-LB) * lambdaBC ... % spring b
        - cB * LBCdot * lambdaBC ... % dashpot b
        - c * rCdot ... % drag
        - m * g * j;  % gravity
end


% STABILITY AND EQUILIBRIUM ANALYSIS
nguesses = 30;
myroots = zeros(4,nguesses^2);
goodroots = zeros(nguesses^2,1);
i = 0;

options = optimoptions('fsolve', 'FunctionTolerance',1e-30, 'OptimalityTolerance',1e-8, 'MaxFunctionEvaluations',10000, 'MaxIterations',10000, 'Disp','off', 'Algorithm', 'levenberg-marquardt');


% STATICS SOL
for xguess = linspace(-31,31,nguesses)
    for yguess = linspace(-31,31,nguesses)
        i = i+1;  % count iterations

        [myroot, fval, exitflag] = fsolve(@(z)myrhs(0,z), [xguess; yguess; 0; 0], options);
        if norm (fval)>1e-6
            disp("Jia says: FSOLVE not happy. we want it to be close to 0")
        else
            goodroots(i) = 1;
        end
        myroots(:,i) = myroot;
    end
end

goodroots = logical(goodroots);
myroots = myroots(:, goodroots);
myroots = round(myroots, 1);
myroots = unique(myroots', 'rows');

disp(['Number of converged optimisations is ' num2str(sum(goodroots))])

nroots = length(myroots(:,1)); % Number of unique roots
disp(['The ' num2str(nroots) ' equilibrium points have these [x; y; vx; vy] values'])
disp('Each row is one root')
disp(myroots)

% PLOTTING EQUILIBRIUM POINTS
figure;
scatter(0,0)
hold on
text(rA(1), rA(2), 'A','FontSize',15)
axis equal
grid on
% xlim([-15,15])
% ylim([-15,15])
text(rB(1), rB(2), 'B','FontSize', 15)
title("Equilibrium points")
xlabel("X axis")
ylabel("Y axis")
shg


% STABILITY ANALYSIS
for i = 1:length(myroots)

    whichroot = i;
    zstar = [myroots(whichroot,:)]';
    scatter(zstar(1), zstar(2), 40, "red",'filled')
    
    h = 1e-4;
    J = zeros(4,4);
    
    for i = 1:4
        zperturb = zeros(4,1);
        zperturb(i) = h;
    
        J(:,i) = (myrhs(0,zstar+zperturb) - myrhs(0,zstar-zperturb))/(2*h);
    end

    [V,D] = eig(J);
    disp(['Root at location: x = ' num2str(zstar(1))  ' y = ' num2str(zstar(2))])
    disp("Eigenvalues: "); disp(diag(D));
end
disp("Eigenvalues should not have positive real part for stability")


