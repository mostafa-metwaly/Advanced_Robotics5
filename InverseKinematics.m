% Solving the inverse Kinematics of the delta robot using the Analytical
% solution and applying the vectors for each joint in the whole manipulator.
function  [theta_angles]=InverseKinematics(params,pose)

[x, y, z] = feval(@(x) x{:}, num2cell(pose));
[L, l, sb, wb, ub, sp, wp, up, h] = feval(@(x) x{:}, num2cell(params));


%solving the relation between the platform and base :
a = wb - up;
b = sp/2 - wb * sqrt(3)/2;
c = wp - 0.5 * wb;

%writing the quadratic equations from the three constraint equations yield the kinematics equation : 
% E*cos(theta) + F*sin(theta)+G = 0,
%  E,F,G for each leg:
E1 = 2*L*(y+a);
F1 = 2*z*L;
G1 = x^2 + y^2 + z^2 + a^2 + L^2 +(2*a*y )- l^2;

E2 = -L * (sqrt(3)*(x+b)+y+c);
F2 = 2*z*L;
G2 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*(x*b + c*y) - l^2;

E3 = L * (sqrt(3)*(x-b)-y-c);
F3 = 2*z*L;
G3 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*( -x*b + c*y) - l^2;


%solving for the elbow up or elbow down(2):
% m = 1;
m = 2;

P1 = [G1-E1 2*F1 G1+E1];
P2 = [G2-E2 2*F2 G2+E2];
P3 = [G3-E3 2*F3 G3+E3];

%solving the quadratic equation for each leg getting two solutions:
t = [roots(P1) roots(P2) roots(P3)];

%  Tangent Half-Angle Substitution:
theta = real([2* atan(t(m,1)); 2* atan(t(m,2)); 2* atan(t(m,3))]);

%Saving the actuated angles of each joint:
theta_angles = [theta(1) theta(2) theta(3)];

end
