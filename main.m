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

radius = 200;
angles = linspace(0, 360, points);
x_circle = radius*sind(angles);
y_circle = radius*cosd(angles);
z_circle = ones(1, points)*-750;
title("Inverse Kinematics Circle Test")
for i=1:points
    if ~ishandle(Robot), return, end
    delete([links_plot,joints_plot, platform_plot])
    delete(axes_plot)
    pose = [x_circle(i) y_circle(i) z_circle(i)];
    q = InverseKinematics(params,pose);
    q = rad2deg(q);
    [x, y, z] = FK(q, params, T_bases, T_tools, 1);
    drawnow
end


%% Singularity Map
% step = 20;
% m = [];
% pose = [];
% tot = ceil(200/20);
% counter = 0;
% for q1 = -20:step:180
%     counter = counter+1;
%     sprintf("Loop %d out of %d loops",counter,tot)
%     for q2 = -20:step:180
%         for q3 = -20:step:180
%             q = [q1 q2 q3];
% % for i=1:points
%         
%             if ~ishandle(Robot), return, end
%             delete([links_plot,joints_plot, platform_plot])
%             delete(axes_plot)
% 
%             [x, y, z] = FK(q, params, T_bases, T_tools, 0);
%             if (isnan(x)) || (isnan(y)) || (isnan(z))
%                 continue
%             end
%             pose(end+1,:) = [x y z];
%             J = Jacobian(x,y,z, q(1), q(2), q(3), params);
%             m(end+1) = round(sqrt(det(J * J')));
% 
%             hold on
%             if (m(end) < 200000 )
%                 plot3(x, y, z,'*','Color','1 0 0 1','MarkerSize',20);
%             else
%                 plot3(x, y, z,'.','Color','0.8 0 0.8 1','MarkerSize',20);
%             end
%             drawnow
%     
%         end
%     end
% end
%% Plotting:

% figure;
% titles = ['x', 'y', 'z'];
% scatter3(pose(:,1), pose(:,2), pose(:,3), 40, m,'filled')    % draw the scatter plot
% ax = gca;
% % ax.XDir = 'reverse';
% %     view(-31,14)
% xlabel("X")
% ylabel("Y")
% zlabel("Z")
% title('Singularity Map')
% 
% colormap(flipud(winter));
% cb = colorbar;                                     % create and label the colorbar
% cb.Label.String = 'Manipulability Magnitude';
% 
% hold on
% FK([0 0 0], params, T_bases, T_tools, 1);