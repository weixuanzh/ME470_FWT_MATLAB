% platform orientation is defined according to ZYZ euler angles (rotate around z by alpha, around y by beta, around z by gamma)
% all rotations are in moving frame
% platform geometry constrains that alpha = -gamma
% only 2 dof in orientation
% not all (yaw, pitch, roll) combination is doable
% if (pitch, roll) is specified, yaw is determined

%% all doable roll and pitch at a given yaw
th_p = -pi/2:pi/300:pi/2;
th_r = -pi/2:pi/300:pi/2;
th_y = pi/4;

figure
hold on
for i = th_p
    for j = th_r
        eul_zyz = rotm2eul(eul2rotm([th_y, j, -i], 'ZYX'), 'ZYZ');
        if abs(eul_zyz(1) + eul_zyz(3)) < 1e-3
            plot(i, j, 'o')
        end
    end
end

xlabel("pitch")
ylabel("roll")

%% all doable (yaw, pitch, roll) combinations
figure
hold on
th_alpha = -pi/2:pi/20:pi/2;
th_beta = -pi/2:pi/20:pi/2;
for i = th_alpha
    for j = th_beta
        eul_zyx = rotm2eul(eul2rotm([i, j, -i], 'ZYZ'), 'ZYX');
        plot3(eul_zyx(1), eul_zyx(2), eul_zyx(3), 'o')
    end
end
xlabel("yaw")
ylabel("pitch")
zlabel("roll")

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
   