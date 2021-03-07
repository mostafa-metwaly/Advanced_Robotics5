clear all
close all
clc

%% Robot parameters (in mm):
% base lengths:
wb = 164;
sb = 567;
ub = 327;

% tool lengths:
up = 44;
sp = 76;
wp = 22;

% links lengths:
L = 524;
l = 1244;
h = 0.05*l;

% Base transformations:
T_base1 = Ty(-wb) * Rz(pi);
T_base2 = Tx(sqrt(3)*wb/2) * Ty(0.5*wb) * Rz(-pi/3);
T_base3 = Tx(-sqrt(3)*wb/2) * Ty(0.5*wb) * Rz(pi/3);
T_bases = {T_base1, T_base2, T_base3};

% Tool transformations:
T_tool1 = Ty(-up) * Rz(pi);
T_tool2 = Tx(sp/2) * Ty(wp) * Rz(-pi/3);
T_tool3 = Tx(-sp/2) * Ty(wp) * Rz(pi/3);
T_tools = {T_tool1, T_tool2, T_tool3};

params = [L, l, sb, wb, ub, sp, wp, up, h];

% q = [0 0 0]; % in degrees
% [x, y, z] = FK(q, params, T_bases, T_tools, 1);

%% Testing Forward Kinematics:
Robot = figure('units','normalized','outerposition',[0 0 1 1]);
global axes_plot links_plot joints_plot end_effector_plot platform_plot
axes_plot = plot3(0,0,0);
hold on
links_plot = plot3(0,0,0);
hold on
joints_plot = plot3(0,0,0);
hold on
end_effector_plot = plot3(0,0,0);
hold on
plt = plot3(0,0,0);

points = 60;

% radius = 200;
% angles = linspace(0, 360, points);
% x_circle = radius*sind(angles);
% y_circle = radius*cosd(angles);
% z_circle = ones(1, points)*-750;
step = 30;
m = [];
pose = [];
for q1 = -30:step:90
    for q2 = -30:step:90
        for q3 = -30:step:90
            q = [q1 q2 q3];
% for i=1:points
        
            if ~ishandle(Robot), return, end
            delete([links_plot,joints_plot, platform_plot])
            delete(axes_plot)

%             [x ,y, z] = deal(0 ,0, -q1*20);
            [x,y,z] = FK(q, params, T_bases, T_tools, 1)
            pose = [x y z];
            q =InverseKinematics(params, pose)
            pose(end+1,:) = [x y z];
            J = Jacobian(x,y,z, q(1), q(2), q(3), params);
            m(end+1) = sqrt(det(J * J'))

            hold on
            if (m < 1 )
                plt = [plt plot3(x, y, z,'.','Color','1 0 0 1','MarkerSize',20)];                
            else
                plt = [plt plot3(x, y, z,'.','Color','0.8 0 0.8 1','MarkerSize',20)];

            end
            drawnow
    
        end
    end
end
