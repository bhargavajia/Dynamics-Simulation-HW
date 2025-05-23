function [A,B] = DAE_matrices(in1,in2)
%DAE_matrices
%    [A,B] = DAE_matrices(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    14-May-2025 00:39:15

I = in2(2,:);
d1 = in2(3,:);
d2 = in2(4,:);
m = in2(1,:);
theta = in1(3,:);
theta1 = in2(5,:);
theta2 = in2(6,:);
thetadot = in1(6,:);
xdot = in1(4,:);
ydot = in1(5,:);
t2 = cos(theta1);
t3 = cos(theta2);
t4 = theta+theta1;
t5 = theta+theta2;
t12 = -m;
t6 = d1.*t2;
t7 = d2.*t3;
t8 = cos(t4);
t9 = cos(t5);
t10 = sin(t4);
t11 = sin(t5);
t13 = -t6;
t14 = -t10;
t15 = -t11;
A = reshape([t12,0.0,0.0,t14,t15,0.0,t12,0.0,t8,t9,0.0,0.0,-I,t13,t7,t14,t8,t13,0.0,0.0,t15,t9,t7,0.0,0.0],[5,5]);
if nargout > 1
    B = [0.0;0.0;0.0;t8.*thetadot.*xdot+t10.*thetadot.*ydot;t9.*thetadot.*xdot+t11.*thetadot.*ydot];
end
end
