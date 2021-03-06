

function  [theta_angles]=InverseKinematics(params,pose)

[x, y, z] = feval(@(x) x{:}, num2cell(pose));
[L, l, sb, wb, ub, sp, wp, up, h] = feval(@(x) x{:}, num2cell(params));

a = wb - up;
b = sp/2 - wb * sqrt(3)/2;
c = wp - 0.5 * wb;


% E*cos(theta) + F*sin(theta)+G=0

E1 = 2*L*(y+a);
F1 = 2*z*L;
G1 = x^2 + y^2 + z^2 + a^2 + L^2 +2*a*y - l^2;

E2 = L * (sqrt(3)*(x+b)+y+c);
F2 = 2*z*L;
G2 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*(x*b + c*y) - l^2;

E3 = L * (sqrt(3)*(x-b)-y-c);
F3 = 2*z*L;
G3 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*( -x*b + c*y) - l^2;

m = -1;

t = [-F1 + m*sqrt(E1^2 + F1^2 - G1^2)/(G1-E1); -F2 + m*sqrt(E3^2 + F2^2 - G2^2)/(G2-E2);-F3 + m*sqrt(E3^2 + F3^2 - G3^2)/(G3-E3)];

%  Tangent Half-Angle Substitution:
theta = real([2* atan(t(1)); 2* atan(t(2)); 2* atan(t(3))]);


theta_angles = [theta(1) theta(2) theta(3)]
end
