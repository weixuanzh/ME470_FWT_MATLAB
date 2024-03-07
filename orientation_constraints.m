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
   