function J = Jacobian(x, y, z, theta1, theta2, theta3, params)

[L, l, sB, wB, uB, sP, wP, uP, h] = feval(@(x) x{:}, num2cell(params));

a = wB - uP ;
b = sP/2 - sqrt(3)/2 * wB ;
c = wP - 1/2 * wB ;

J_P = [x,                             y+a+L*cos(theta1),     z+L*sin(theta1);
       2*(x+b)-sqrt(3)*L*cos(theta2), 2*(y+c)-L*cos(theta2), 2*(z+L*sin(theta2));
       2*(x-b)+sqrt(3)*L*cos(theta3), 2*(y+c)-L*cos(theta3), 2*(z+L*sin(theta3))] ;

b11 = L*((y+a)*sin(theta1) - z*cos(theta1)) ;
b22 = -L*((sqrt(3)*(x+b)+y+c)*sin(theta2) + 2*z*cos(theta2)) ;
b33 = L*((sqrt(3)*(x-b)-y-c)*sin(theta3) - 2*z*cos(theta3)) ;

J_theta = [b11 0    0;
           0   b22  0;
           0   0   b33];
       
J = inv(J_P) * J_theta ;

end