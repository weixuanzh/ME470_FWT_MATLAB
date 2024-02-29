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
        