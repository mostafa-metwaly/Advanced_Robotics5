% Author ~ Ahmed Magd Aly
% Innopolis University

function [x, y, z] = FK(q, params, T_bases, T_tools, enable_plot)

q = deg2rad(q);

[q1, q2, q3] = feval(@(x) x{:}, num2cell(q));
[L, l, sb, wb, ub, sp, wp, up, h] = feval(@(x) x{:}, num2cell(params));

A1v = [0; -wb - L*cos(q1) + up; -L*sin(q1)];
A2v = [sqrt(3)*(wb+L*cos(q2))/2 - sp/2; (wb+L*cos(q2))/2 - wp; -L*sin(q2)];
A3v = [-sqrt(3)*(wb+L*cos(q3))/2 + sp/2; (wb+L*cos(q3))/2 - wp; -L*sin(q3)];

p = [A1v, A2v, A3v];
result = interx(p(:,1),p(:,2),p(:,3),l,l,l,0);
[x, y, z] = feval(@(x) x{:}, num2cell(result));
if(isnan(x)) || (isnan(y)) || (isnan(z)), return; end
%% Plotting

global axes_plot links_plot joints_plot platform_plot

nVector = [0 0 1]; % vector parallel to base plane
V0 = [0 0 0]; % Arbitrary point on the base plane

R = [pi, -pi/3, pi/3];
for i = 1:3
    
    
    OT = Tx(x) * Ty(y) * Tz(z) * cell2mat(T_tools(i));

    O0 = cell2mat(T_bases(i));
    O1 = O0 * Rx(-q(i)) * Ty(L);
    O2 = O1 * Tx(h/2);
    
    P0 = [O1(1,4) O1(2,4) O1(3,4)]; % point 1 on the parallelogram link
    P1 = [OT(1,4) OT(2,4) OT(3,4)]; % point 2 on the parallelogram link
    
    [I,check]=plane_line_intersect(nVector,V0,P0,P1);
    
    if check == 0 || check == 2
        x = nan; y = nan; z = nan;
        return
    end
    
    xy = sqrt(I(1)^2 + I(2)^2);
    
    if xy <= wb
        x = nan; y = nan; z = nan;
        disp("intersecting the base plane")
        return
    end
    
    OT = OT * Tx(h/2);
    
    T_parallel = [OT(1,4) - O2(1,4), OT(2,4) - O2(2,4), OT(3,4) - O2(3,4)];
    T_active = [O1(1,4) - O0(1,4), O1(2,4) - O0(2,4), O1(3,4) - O0(3,4)];
    T_parallel = T_parallel*l/norm(T_parallel);
    
    
    passive_joint1_phi = atan2(T_parallel(1), T_parallel(2));
    passive_joint1_theta = atan2(T_parallel(3), T_parallel(2));
    active_angle = atan2(T_active(3), T_active(2));
    
    angle_parallel(i) = passive_joint1_theta - active_angle;
    
%     passive_joint2_phi = atan2(T_parallel(1), T_parallel(2));
%     passive_joint2_theta = atan2(T_parallel(1), T_parallel(2));
    
    O3 = O2 * Rx(q(i)) * Rz(-R(i)) * Tx( T_parallel(1) ) * Ty( T_parallel(2) ) * Tz( T_parallel(3) );
    O4 = O3 * Rz(R(i)) * Tx(-h/2);
    O5 = O4 * Tx(-h/2);
    O6 = O5 * Rz(-R(i)) * Tx( -T_parallel(1) ) * Ty( -T_parallel(2) ) * Tz( -T_parallel(3) );
    O7 = O6 * Rz(R(i)) * Tx(h/2);

    O_all(:,:,i) = [O0 O1 O2 O3 O4 O5 O6 O7];
end

% checking = angle_parallel > 0;
% if sum(checking) > 0
%     O_all = []
%     result = interx(p(:,1),p(:,2),p(:,3),l,l,l,0);
%     [x, y, z] = feval(@(x) x{:}, num2cell(result));
%     
%     for i = 1:3
%         OT = Tx(x) * Ty(y) * Tz(z) * cell2mat(T_tools(i));
% 
%         O0 = cell2mat(T_bases(i));
%         O1 = O0 * Rx(-q(i)) * Ty(L);
%         O2 = O1 * Tx(h/2);
% 
%         P0 = [O1(1,4) O1(2,4) O1(3,4)]; % point 1 on the parallelogram link
%         P1 = [OT(1,4) OT(2,4) OT(3,4)]; % point 2 on the parallelogram link
% 
%         [I,check]=plane_line_intersect(nVector,V0,P0,P1);
% 
%         if check == 0 || check == 2
%             x = nan; y = nan; z = nan;
%             return
%         end
% 
%         xy = sqrt(I(1)^2 + I(2)^2);
% 
%         if xy <= wb
%             x = nan; y = nan; z = nan;
%             disp("intersecting the base plane")
%             return
%         end
% 
%         OT = OT * Tx(h/2);
% 
%         T_parallel = [OT(1,4) - O2(1,4), OT(2,4) - O2(2,4), OT(3,4) - O2(3,4)];
%         T_active = [O1(1,4) - O0(1,4), O1(2,4) - O0(2,4), O1(3,4) - O0(3,4)];
%         T_parallel = T_parallel*l/norm(T_parallel);
% 
% 
%         passive_joint1_phi = atan2(T_parallel(1), T_parallel(2));
%         passive_joint1_theta = atan2(T_parallel(3), T_parallel(2));
%         active_angle = atan2(T_active(3), T_active(2));
% 
%         angle_parallel(i) = passive_joint1_theta - active_angle;
% 
%     %     passive_joint2_phi = atan2(T_parallel(1), T_parallel(2));
%     %     passive_joint2_theta = atan2(T_parallel(1), T_parallel(2));
% 
%         O3 = O2 * Rx(q(i)) * Rz(-R(i)) * Tx( T_parallel(1) ) * Ty( T_parallel(2) ) * Tz( T_parallel(3) );
%         O4 = O3 * Rz(R(i)) * Tx(-h/2);
%         O5 = O4 * Tx(-h/2);
%         O6 = O5 * Rz(-R(i)) * Tx( -T_parallel(1) ) * Ty( -T_parallel(2) ) * Tz( -T_parallel(3) );
%         O7 = O6 * Rz(R(i)) * Tx(h/2);
% 
%         O_all(:,:,i) = [O0 O1 O2 O3 O4 O5 O6 O7];
%     end
% end

for i = 1:3
    O = O_all(:,:,i);
    if enable_plot
        % figure('units','normalized','outerposition',[0 0 1 1])



        as = 0.000150; % axes scaler
        color = ['r','g','b']; % axes color

        index = 0;
        for i = 1:4:length(O)
            index = index + 1;
            points_x(index) = O(1,i+3);
            points_y(index) = O(2,i+3);
            points_z(index) = O(3,i+3);

            if index ~= 2 && index ~= 5
                for axes = 0:2

                    axes_plot = [axes_plot plot3([O(1,i+3) as*O(1,i+axes)+O(1,i+3)], [O(2,i+3) as*O(2,i+axes)+O(2,i+3)], [O(3,i+3) as*O(3,i+axes)+O(3,i+3)],'Color',color(axes+1))];
                    hold on
                end
            end
        end
        joints_x = points_x;
        joints_y = points_y;
        joints_z = points_z;


        links_plot = [links_plot plot3(points_x, points_y, points_z,'Color', "0 0 0",'linewidth',2)];
        hold on
        joints_plot = [joints_plot plot3(joints_x, joints_y, joints_z,'.','Color','0.992 0.788 0.04 1','MarkerSize',12.5)];
        hold on
%         end_effector_plot = [end_effector_plot plot3(points_x(5), points_y(5), points_z(5),'.','Color','0 0.8 0 1','MarkerSize',12.5)];
%         path_plot = plot3(joints_x(6), joints_y(6), joints_z(6),'.','Color','0.8 0 0 1','MarkerSize',7.5);


        xlim([-1500 1500])
        ylim([-1500 1500])
        zlim([-1500 1500])
%         view(0,90)
        grid on
        xlabel("X - Axis")
        ylabel("Y - Axis")
        zlabel("Z - Axis")
    end
    
    nPlatformPoints = 20;
    platform_radius = up;
    circle_angles = linspace(0,360,nPlatformPoints);
    platform_x = x + platform_radius * sind(circle_angles);
    platform_y = y + platform_radius * cosd(circle_angles);
    platform_z = ones(1,nPlatformPoints)*z;
    
    platform_plot = [platform_plot fill3(platform_x, platform_y, platform_z,'red')];
    
    nBasePoints = 20;
    Base_radius = wb;
    circle_angles = linspace(0,360,nBasePoints);
    Base_x = Base_radius * sind(circle_angles);
    Base_y = Base_radius * cosd(circle_angles);
    Base_z = zeros(1,nBasePoints);
    
     platform_plot = [platform_plot fill3(Base_x, Base_y, Base_z,'black')];
    
    plot3(x, y, z,'.','Color','0.8 0 0 1','MarkerSize',12.5);
    
end

