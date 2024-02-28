% reference trajectory parameters
pitch_freq = 2;
pitch_amp = pi/6;
pitch_offset = 0;
roll_freq = 1;
roll_amp = pi/6;
roll_offset = 0;
z_center_freq = 5;
z_center_amp = 50;
z_center_offset = 400;

% simulation parameters
delta_t = 0.01;
terminal_t = 5;

% platform parameters
pin_distance = 173;
ball_distance = 173;

% generate reference trajectory
time = 0:delta_t:terminal_t;
pitch_ref = pitch_amp .* sin(2 * pi * pitch_freq * time) + pitch_offset;
roll_ref = roll_amp .* sin(2 * pi * roll_freq * time) + roll_offset;
z_ref = z_center_amp .* sin(2 * pi * z_center_freq * time) + z_center_offset;
nsteps = length(time);
d_history = zeros(3, nsteps);

alpha_ref = zeros(nsteps, 1);
beta_ref = zeros(nsteps, 1);

for i = 1:nsteps
    c1 = cos(pitch_ref(i));
    s1 = sin(pitch_ref(i));
    c2 = cos(roll_ref(i));
    s2 = sin(roll_ref(i));
    rotm = [1, 0, 0; 0, c2, -s2; 0, s2, c2] * [c1, 0, -s1; 0, 1, 0; s1, 0, c1];
    eulZYZ = rotm2eul(rotm, 'ZYZ');
    alpha_ref(i) = eulZYZ(1);
    beta_ref(i) = eulZYZ(2);
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
    [th1, th2, th3] = RPS_forward_kinematics(d_history(1, i), d_history(2, i),d_history(2, i), ball_distance, pin_distance, prev_guess);
    prev_guess = [th1; th2; th3];
    th1_history(i) = th1;
    th2_history(i) = th2;
    th3_history(i) = th3;
end

%% Plot pin angles
figure
hold on
plot(time, th1_history * 180 / pi - 90)
plot(time, th2_history * 180 / pi - 90)
plot(time, th3_history * 180 / pi - 90)
xlabel("time (s)")
ylabel("pin angle from vertical (degree)")
legend(["\theta_1", "\theta_2", "\theta_3"])

%% animate platform motion
animation_length = 4;
t_step_plot = animation_length / nsteps;
z_history = zeros(nsteps, 1);
% 1 is for recording the motion video, 0 otherwise
record = 0;

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
    vp = 2;
    z_temp = RPS_plotting(d_history(1, i), d_history(2, i),d_history(3, i), th1_history(i), th2_history(i), th3_history(i), ball_distance, pin_distance, vp);
    z_history(i) = z_temp;
    drawnow
    if record
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
end
if record
    close(myVideo)
end
% plot z center
figure
plot(time, z_history)
xlabel("time (s)")
ylabel("center height")