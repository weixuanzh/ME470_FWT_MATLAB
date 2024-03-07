% reference trajectory parameters
pitch_freq = 1;
pitch_amp = 20 * pi / 180;
pitch_offset = 0;
roll_freq = 3;
roll_amp = 20 * pi / 180;
roll_offset = 0;
z_center_freq = 1;
z_center_amp = 0;
z_center_offset = 203.835;

% simulation parameters
delta_t = 0.005;
terminal_t = 2;

% platform parameters
pin_distance = 83.2358;
ball_distance = 86.614;

% generate reference trajectory
time = 0:delta_t:terminal_t;
pitch_ref = pitch_amp .* sin(2 * pi * pitch_freq * time - pi/4) + pitch_offset;
roll_ref = roll_amp .* sin(2 * pi * roll_freq * time) + roll_offset;
z_ref = z_center_amp .* sin(2 * pi * z_center_freq * time) + z_center_offset;
nsteps = length(time);
d_history = zeros(3, nsteps);
yaw_history = zeros(1, nsteps);

% convert (yaw, pitch, roll) (ZYX euler angle) to ZYZ euler angle (alpha, beta, -alpha)
% yaw angle is dependent on provided pitch and roll angles
alpha_ref = zeros(nsteps, 1);
beta_ref = zeros(nsteps, 1);

for i = 1:nsteps
    c1 = cos(pitch_ref(i));
    s1 = sin(pitch_ref(i));
    c2 = cos(roll_ref(i));
    s2 = sin(roll_ref(i));
    % rotation matrix for pitch and roll
    % pitch first, roll second, in moving frame
    rotm = [c1, 0, -s1; 0, 1, 0; s1, 0, c1] * [1, 0, 0; 0, c2, -s2; 0, s2, c2];
    % solve for yaw angle
    th_yaw = atan((rotm(1, 2) - rotm(2, 1)) / (rotm(1, 1) + rotm(2, 2)));
    rotm = [cos(th_yaw), -sin(th_yaw), 0; sin(th_yaw), cos(th_yaw), 0; 0, 0, 1] * rotm;
    eulZYZ = rotm2eul(rotm, 'ZYZ');
    alpha_ref(i) = eulZYZ(1);
    beta_ref(i) = eulZYZ(2);
    yaw_history(i) = th_yaw;
end

% solve IK at all time steps
for i = 1:nsteps
    [d1, d2, d3] = RPS_inverse_kinematics(z_ref(i), alpha_ref(i), beta_ref(i), ball_distance, pin_distance);
    d_history(1, i) = d1;
    d_history(2, i) = d2;
    d_history(3, i) = d3;
end

%% plot actuator length trajectory from IK
figure
hold on
plot(time, d_history(1, :))
plot(time, d_history(2, :))
plot(time, d_history(3, :))
xlabel("time (s)")
ylabel("actuator length (mm)")
legend(["d1", "d2", "d3"])

%% Forward kinematics and plotting

% initial guess for forward kinematics
prev_guess = [pi/2; pi/2; pi/2];
% prev_guess = [0; 0; 0];
th1_history = zeros(nsteps, 1);
th2_history = zeros(nsteps, 1);
th3_history = zeros(nsteps, 1);
% prepare the plot

for i = 1:nsteps
    [th1, th2, th3] = RPS_forward_kinematics(d_history(1, i), d_history(2, i),d_history(3, i), ball_distance, pin_distance, prev_guess);
    %prev_guess = [th1; th2; th3];
    th1_history(i) = th1;
    th2_history(i) = th2;
    th3_history(i) = th3;
end

%% Plot pin angles and platform yaw
figure
hold on
plot(time, th1_history * 180 / pi - 90)
plot(time, th2_history * 180 / pi - 90)
plot(time, th3_history * 180 / pi - 90)
xlabel("time (s)")
ylabel("pin angle from vertical (degree)")
legend(["\theta_1", "\theta_2", "\theta_3"])
figure
plot(time, yaw_history * 180 / pi)
xlabel("time (s)")
ylabel("platform yaw (degree)")

%% animate platform motion
animation_length = 4;
t_step_plot = animation_length / nsteps;
z_history = zeros(nsteps, 1);
x_history = zeros(nsteps, 1);
y_history = zeros(nsteps, 1);
xp1_history = zeros(nsteps, 1);
yp1_history = zeros(nsteps, 1);
zp1_history = zeros(nsteps, 1);

xp2_history = zeros(nsteps, 1);
yp2_history = zeros(nsteps, 1);
zp2_history = zeros(nsteps, 1);

xp3_history = zeros(nsteps, 1);
yp3_history = zeros(nsteps, 1);
zp3_history = zeros(nsteps, 1);

% 1 is for recording the motion video, 0 otherwise
record = 0;
% view point: 0 for side view, 1 for front view, 2 for 45 degree view, 3
% for top view

vp = 3;

motion = figure;
tic
if record
    % Initialize video
    myVideo = VideoWriter('myVideoFile');
    myVideo.FrameRate = 10;
    open(myVideo)
end
for i = 1:nsteps
    pause(t_step_plot - toc)
    tic
    % view point: 0 for side view, 1 for front view, 2 for 45 degree view
    [x_temp, y_temp, z_temp, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3] = RPS_plotting(d_history(1, i), d_history(2, i),d_history(3, i), th1_history(i), th2_history(i), th3_history(i), ball_distance, pin_distance, vp);
    z_history(i) = z_temp;
    x_history(i) = x_temp;
    y_history(i) = y_temp;
    xp1_history(i) = xp1;
    yp1_history(i) = yp1;
    zp1_history(i) = zp1;
    
    xp2_history(i) = xp2;
    yp2_history(i) = yp2;
    zp2_history(i) = zp2;
    
    xp3_history(i) = xp3;
    yp3_history(i) = yp3;
    zp3_history(i) = zp3;
    drawnow
    if record
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
end
if record
    close(myVideo)
end
hold on
plot3(xp1_history, yp1_history, zp1_history)
plot3(xp2_history, yp2_history, zp2_history)
plot3(xp3_history, yp3_history, zp3_history)


% plot center
figure
plot(time, x_history)
xlabel("time (s)")
ylabel("center position (mm)")

%% Plot ball joint trajectory
figure
hold on
plot3(xp1_history, yp1_history, zp1_history)
plot3(xp2_history, yp2_history, zp2_history)
plot3(xp3_history, yp3_history, zp3_history)
plot3(x_history, y_history, z_history)

legend(["joint 1", "joint 2", "joint3", "center"])