% platform orientation is defined according to ZYZ euler angles (rotate around z by alpha, around y by beta, around z by gamma)
% all rotations are in moving frame
% platform geometry constrains that alpha = -gamma
% only 2 dof in orientation
% not all (yaw, pitch, roll) combination is doable
% if (pitch, roll) is specified, yaw is determined

%% all doable roll and pitch at a given yaw
th_p = -pi/4:pi/300:pi/4;
th_r = -pi/4:pi/300:pi/4;
th_y = 4 * pi / 180;

figure
hold on
for i = th_p
    for j = th_r
        eul_zyz = rotm2eul(eul2rotm([th_y, j, -i], 'ZYX'), 'ZYZ');
        if abs(eul_zyz(1) + eul_zyz(3)) < 1e-3
            plot(i * 180 / pi, j * 180 / pi, 'o')
        end
    end
end

xlabel("pitch")
ylabel("roll")

%% all doable (yaw, pitch, roll) combinations
figure
hold on
th_alpha = -pi/2:pi/50:pi/2;
th_beta = -pi/2:pi/50:pi/2;
for i = th_alpha
    for j = th_beta
        eul_zyx = rotm2eul(eul2rotm([i, j, -i], 'ZYZ'), 'ZYX');
        plot3(eul_zyx(1) * 180 / pi, eul_zyx(2) * 180 / pi, eul_zyx(3) * 180 / pi, 'o')
    end
end
xlabel("yaw (degree)")
ylabel("pitch (degree)")
zlabel("roll (degree)")

% see possible (pitch, roll) pairs when yaw is within [-thres, thres]
pitch_bound = 20;
roll_bound = 20;
pt1 = [0; pitch_bound; roll_bound];
pt2 = [0; pitch_bound; -roll_bound];
pt3 = [0; -pitch_bound; -roll_bound];
pt4 = [0; -pitch_bound; roll_bound];
plot3([pt1(1), pt2(1)], [pt1(2), pt2(2)], [pt1(3), pt2(3)], 'g', 'Linewidth', 4)
plot3([pt2(1), pt3(1)], [pt2(2), pt3(2)], [pt2(3), pt3(3)], 'g', 'Linewidth', 4)
plot3([pt3(1), pt4(1)], [pt3(2), pt4(2)], [pt3(3), pt4(3)], 'g', 'Linewidth', 4)
plot3([pt4(1), pt1(1)], [pt4(2), pt1(2)], [pt4(3), pt1(3)], 'g', 'Linewidth', 4)

thres = 4;
xlim([-thres, thres])
view(90, 0)
title("Possible orientations when yaw is less than " + num2str(thres) + " degrees")

%% calculate actuator rotation angle as a function of (pitch, roll) at a given center height
z_fixed = 203.835;
pin_distance = 83.2358;
ball_distance = 86.614;

% setup roll and pitch
roll_angles = -pi/4:pi/50:pi/4;
pitch_angles = -pi/4:pi/50:pi/4;
figure
hold on
prev_guess = [pi/2, pi/2, pi/2];
for i = roll_angles
    for j = pitch_angles
        % first solve for yaw angles
        % based on rotation matrix constraints
        c1 = cos(j);
        s1 = sin(j);
        c2 = cos(i);
        s2 = sin(i);
        rotm = [c1, 0, -s1; 0, 1, 0; s1, 0, c1] * [1, 0, 0; 0, c2, -s2; 0, s2, c2];
        % solve for yaw angle
        th_yaw = atan((rotm(1, 2) - rotm(2, 1)) / (rotm(1, 1) + rotm(2, 2)));
        rotm = [cos(th_yaw), -sin(th_yaw), 0; sin(th_yaw), cos(th_yaw), 0; 0, 0, 1] * rotm;
        % euler angles in ZYZ sequence
        eulZYZ = rotm2eul(rotm, 'ZYZ');
        alpha = eulZYZ(1);
        beta = eulZYZ(2);
        
        % solve for linkage lengths
        [d1, d2, d3] = RPS_inverse_kinematics(z_fixed, alpha, beta, ball_distance, pin_distance);
        % solve for actuator pin rotation angles
        [th1, th2, th3] = RPS_forward_kinematics(d1, d2, d3, ball_distance, pin_distance, prev_guess);
        max_angle = max(abs([th1, th2, th3]));
        % plot (pitch, roll, actuator angle)
        plot3(j * 180 / pi, i * 180 / pi, max_angle * 180 / pi - 90, 'o')
    end
end
xlabel("pitch angle (degree)")
ylabel("roll angles (degree)")
zlabel("actuator angle from vertical (degree)")
title("Center height at Z=" + num2str(z_fixed) + "mm")

%% Uncontrollable motion: see possible xy translation and yaw when a certain range of pitch and roll are applied
pitch_amp = 20 * pi / 180;
roll_amp = 20 * pi / 180;
step = pi / 500;
pitch_nsteps = ceil(pitch_amp / step);
roll_nsteps = ceil(roll_amp / step);
pitch_vals = 0:pitch_nsteps;
roll_vals = 0:roll_nsteps;
% platform parameters
g = 83.2358;
h = 86.614;
% 4 plots: x, y, yaw as a function of specified dof; all possible (x, y)
% figure
% hold on 
% subplot(2, 3, 1)
% store the resulting uncontrollable yaw and distance
yaw_matrix = zeros(pitch_nsteps, roll_nsteps);
dist_matrix = zeros(pitch_nsteps, roll_nsteps);
x_matrix = zeros(pitch_nsteps, roll_nsteps);
y_matrix = zeros(pitch_nsteps, roll_nsteps);
for p = pitch_vals
    for r = roll_vals
        % calculate required yaw given pitch and roll using rotation matrix
        c1 = cos(p * step);
        s1 = sin(p * step);
        c2 = cos(r * step);
        s2 = sin(r * step);
        % rotation matrix for pitch and roll
        % pitch first, roll second, in moving frame
        rotm = [c1, 0, -s1; 0, 1, 0; s1, 0, c1] * [1, 0, 0; 0, c2, -s2; 0, s2, c2];
        y = atan((rotm(1, 2) - rotm(2, 1)) / (rotm(1, 1) + rotm(2, 2)));
        % convert roll, pitch, yaw into ZYZ euler angles
        rotm = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1] * rotm;
        eulZYZ = rotm2eul(rotm, 'ZYZ');
        alpha = eulZYZ(1);
        beta = eulZYZ(2);
        % calculation xy translation
        px = -0.5 * h * (1 - cos(beta)) * cos(2 * alpha);
        py = 0.5 * h * (1 - cos(beta)) * sin(2 * alpha);
        
        yaw_matrix(pitch_nsteps - (p) + 1, r + 1) = abs(y) * 180 / pi;
        dist_matrix(pitch_nsteps - (p) + 1, r + 1) = sqrt(px^2 + py^2);
        x_matrix(pitch_nsteps - (p) + 1, r + 1) = abs(px);
        y_matrix(pitch_nsteps - (p) + 1, r + 1) = abs(py);
%         subplot(2, 3, 1)
%         hold on
%         plot3(p, r, px, 'o')
%         subplot(2, 3, 2)
%         hold on
%         plot3(p, r, py, 'o')
%         subplot(2, 3, 3)
%         hold on
%         plot3(p, r, y, 'o')
%         subplot(2, 3, 4)
%         hold on
%         plot(px, py, 'o')
%         subplot(2, 3, 5)
%         hold on
%         plot3(p, r, sqrt(px^2 + py^2), 'o')
    end
end
% label the plots
% subplot(2, 3, 1)
% title("x-center")
% xlabel("pitch")
% ylabel("roll")
% zlabel("x-center")
% subplot(2, 3, 2)
% title("y-center")
% xlabel("pitch")
% ylabel("roll")
% zlabel("y-center")
% subplot(2, 3, 3)
% title("yaw")
% xlabel("pitch")
% ylabel("roll")
% zlabel("yaw")
% subplot(2, 3, 4)
% title("Center positions")
% xlabel("x-center")
% ylabel("y-center")
% subplot(2, 3, 5)
% title("Center offset distance from origin")
% xlabel("pitch")
% ylabel("roll")
% zlabel("Offset Distance")

% heatmap yaw and dist
img_temp = imagesc(yaw_matrix);
colormap(hsv(512))
colormap turbo
tmp = colorbar;
tmp.Label.String = 'Platform Yaw (deg)';

% rescale and label axis
xticklocs = [0 10 20 30 40 50];
xticks(xticklocs);
yticklocs = 0:5:60;
yticks(yticklocs);
myfac     = 1 * step * 180 / pi;  % this would be your 12.5578
xticklabels(cellstr(num2str(xticklocs'*myfac)));
xlabel("Platform Roll (deg)")
yticklabels(cellstr(num2str(yticklocs'*myfac)));
ylabel("Platform Pitch (deg)")
% reverse the axis
% get the handle to the parent axes
hAxs = get(img_temp,'Parent');
% reverse the order of the y-axis tick labels
yAxisTickLabels = get(hAxs, 'YTickLabel');
set(hAxs,'YTickLabel',flipud(yAxisTickLabels));
title("Yaw vs. Commanded Orientations")


figure
img_temp = imagesc(dist_matrix);
colormap(hsv(512))
colormap turbo
tmp = colorbar;
tmp.Label.String = 'Center Offset (mm)';
xticks(xticklocs);
xticklabels(cellstr(num2str(xticklocs'*myfac)));
xlabel("Platform Roll (deg)")
yticks(yticklocs);
yticklabels(cellstr(num2str(yticklocs'*myfac)));
ylabel("Platform Pitch (deg)")
% reverse the axis
% get the handle to the parent axes
hAxs = get(img_temp,'Parent');
% reverse the order of the y-axis tick labels
yAxisTickLabels = get(hAxs, 'YTickLabel');
set(hAxs,'YTickLabel',flipud(yAxisTickLabels));
title("Center Offset vs. Commanded Orientations")

figure
img_temp = imagesc(x_matrix);
colormap(hsv(512))
colormap turbo
tmp = colorbar;
tmp.Label.String = '|x Offset| (mm)';
xticks(xticklocs);
xticklabels(cellstr(num2str(xticklocs'*myfac)));
xlabel("Platform Roll (deg)")
yticks(yticklocs);
yticklabels(cellstr(num2str(yticklocs'*myfac)));
ylabel("Platform Pitch (deg)")
% reverse the axis
% get the handle to the parent axes
hAxs = get(img_temp,'Parent');
% reverse the order of the y-axis tick labels
yAxisTickLabels = get(hAxs, 'YTickLabel');
set(hAxs,'YTickLabel',flipud(yAxisTickLabels));
title("x Offset vs. Commanded Orientations")

figure
img_temp = imagesc(y_matrix);
colormap(hsv(512))
colormap turbo
tmp = colorbar;
tmp.Label.String = '|y Offset| (mm)';
xticks(xticklocs);
xticklabels(cellstr(num2str(xticklocs'*myfac)));
xlabel("Platform Roll (deg)")
yticks(yticklocs);
yticklabels(cellstr(num2str(yticklocs'*myfac)));
ylabel("Platform Pitch (deg)")
% reverse the axis
% get the handle to the parent axes
hAxs = get(img_temp,'Parent');
% reverse the order of the y-axis tick labels
yAxisTickLabels = get(hAxs, 'YTickLabel');
set(hAxs,'YTickLabel',flipud(yAxisTickLabels));
title("y Offset vs. Commanded Orientations")