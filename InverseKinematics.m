

function  [theta_angles]=InverseKinematics(params,pose)

[x, y, z] = feval(@(x) x{:}, num2cell(pose));
[L, l, sb, wb, ub, sp, wp, up, h] = feval(@(x) x{:}, num2cell(params));

a = wb - up;
b = sp/2 - wb * sqrt(3)/2;
c = wp - 0.5 * wb;

% [x,y,z] = deal(00,1000,-1000);
% E*cos(theta) + F*sin(theta)+G=0

E1 = 2*L*(y+a);
F1 = 2*z*L;
G1 = x^2 + y^2 + z^2 + a^2 + L^2 +(2*a*y )- l^2;

E2 = -L * (sqrt(3)*(x+b)+y+c);
F2 = 2*z*L;
G2 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*(x*b + c*y) - l^2;

E3 = L * (sqrt(3)*(x-b)-y-c);
F3 = 2*z*L;
G3 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*( -x*b + c*y) - l^2;

m = 2;
% m = 2;

Px = [G1-E1 2*F1 G1+E1];
Py = [G2-E2 2*F2 G2+E2];
Pz = [G3-E3 2*F3 G3+E3];
t = [roots(Px) roots(Py) roots(Pz)];
%  Tangent Half-Angle Substitution:
theta = real([2* atan(t(m,1)); 2* atan(t(m,2)); 2* atan(t(m,3))]);

theta_angles = [theta(1) theta(2) theta(3)];

end
