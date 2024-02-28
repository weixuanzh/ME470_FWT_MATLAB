%%
threshold = 1e-5;

% distance between the center of the pin and centerline of motor
pin_offset = -4.4695;
% distance between each mounting point
mount_dist = 15;
% distance between motor center and fully retracted end
motor_retracted = 8.42;
% distance between pin joint centers
pin_dist = mount_dist - 2 * cos(pi/6) * pin_offset;
% distance between formation center and pin
pin_center_dist = (pin_dist/2) / cos(pi/6);
% distance between formation center and mounting point
mount_center_dist = (mount_dist/2) / cos(pi/6);

% joint variables for 1 leg
% tp: angle of pin, 0 it the motor is vertical
% dm: extended length of the motor
% tbx, tby, tbz: angle of the ball joint (mounted to platform)
syms tp dm tbx tby tbz 

% HTM from earth frame to pin frame, centered at pin center
% parameter naming: theta_i_j: theta_j for leg i
theta_1_1 = pi/3;
d_1_1 = 0;
a_1_1 = pin_center_dist;
alpha_1_1 = pi/2;
H_1_01 = [cos(theta_1_1), -sin(theta_1_1) * cos(alpha_1_1), sin(theta_1_1) * sin(alpha_1_1), a_1_1 * cos(theta_1_1);
         sin(theta_1_1), cos(theta_1_1) * cos(alpha_1_1), -cos(theta_1_1) * sin(alpha_1_1), a_1_1 * sin(theta_1_1);
         0, sin(alpha_1_1), cos(alpha_1_1), d_1_1;
         0, 0, 0, 1];
     
% HTM from pin frame to motor center frame
theta_1_2 = tp;
d_1_2 = 0;
a_1_2 = pin_offset;
alpha_1_2 = 0;
H_1_12 = [cos(theta_1_2), -sin(theta_1_2) * cos(alpha_1_2), sin(theta_1_2) * sin(alpha_1_2), a_1_2 * cos(theta_1_2);
         sin(theta_1_2), cos(theta_1_2) * cos(alpha_1_2), -cos(theta_1_2) * sin(alpha_1_2), a_1_2 * sin(theta_1_2);
         0, sin(alpha_1_2), cos(alpha_1_2), d_1_2;
         0, 0, 0, 1];
     
% HTM from motor center frame to frame connected to ball joint
% model the ball joint as 3 pin joints stacked on top of each other
% intermediate HTM 1
% rotation in the same direction as z in frame 1
theta_1_31 = pi/2;
d_1_31 = 0;
a_1_31 = motor_retracted + dm;
alpha_1_31 = 0;
H_1_231 = [cos(theta_1_31), -sin(theta_1_31) * cos(alpha_1_31), sin(theta_1_31) * sin(alpha_1_31), a_1_31 * cos(theta_1_31);
          sin(theta_1_31), cos(theta_1_31) * cos(alpha_1_31), -cos(theta_1_31) * sin(alpha_1_31), a_1_31 * sin(theta_1_31);
          0, sin(alpha_1_31), cos(alpha_1_31), d_1_31;
          0, 0, 0, 1];

% intermediate HTM 2
theta_1_32 = tbz;
d_1_32 = 0;
a_1_32 = 0;
alpha_1_32 = pi/2;
H_1_3132 = [cos(theta_1_32), -sin(theta_1_32) * cos(alpha_1_32), sin(theta_1_32) * sin(alpha_1_32), a_1_32 * cos(theta_1_32);
           sin(theta_1_32), cos(theta_1_32) * cos(alpha_1_32), -cos(theta_1_32) * sin(alpha_1_32), a_1_32 * sin(theta_1_32);
           0, sin(alpha_1_32), cos(alpha_1_32), d_1_32;
           0, 0, 0, 1];
     
% intermediate HTM 3
theta_1_33 = tbx - pi/2;
d_1_33 = 0;
a_1_33 = 0;
alpha_1_33 = -pi/2;
H_1_3233 = [cos(theta_1_33), -sin(theta_1_33) * cos(alpha_1_33), sin(theta_1_33) * sin(alpha_1_33), a_1_33 * cos(theta_1_33);
           sin(theta_1_33), cos(theta_1_33) * cos(alpha_1_33), -cos(theta_1_33) * sin(alpha_1_33), a_1_32 * sin(theta_1_33);
           0, sin(alpha_1_33), cos(alpha_1_33), d_1_33;
           0, 0, 0, 1];
     
% HTM from frame 33 to center of top platform
theta_1_4 = tby;
d_1_4 = 0;
a_1_4 = mount_center_dist;
alpha_1_4 = 0;
H_1_334 = [cos(theta_1_4), -sin(theta_1_4) * cos(alpha_1_4), sin(theta_1_4) * sin(alpha_1_4), a_1_4 * cos(theta_1_4);
           sin(theta_1_4), cos(theta_1_4) * cos(alpha_1_4), -cos(theta_1_4) * sin(alpha_1_4), a_1_4 * sin(theta_1_4);
           0, sin(alpha_1_4), cos(alpha_1_4), d_1_4;
           0, 0, 0, 1];
% HTM from earth frame to link frames
H_1_02 = vpa(simplify(H_1_01 * H_1_12));
H_1_02 = mapSymType(H_1_02, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
H_1_03 = vpa(simplify(H_1_02 * H_1_231));
H_1_03 = mapSymType(H_1_03, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
H_1_04 = vpa(simplify(H_1_03 * H_1_3132 * H_1_3233 * H_1_334));
H_1_04 = mapSymType(H_1_04, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));

% pin of leg 1 in world frame       
p_pin_1 = H_1_01 * [0; 0; 0; 1];

% motor center of leg 1 in world frame
p_motor_1 = vpa(simplify(H_1_02 * [0; 0; 0; 1]));
% ball joint of leg 1 in world frame
p_ball_1 = vpa(simplify(H_1_03 * [0; 0; 0; 1])); 
% center of platform leg 1 in world frame
p_center_1 = vpa(simplify(H_1_04 * [0; 0; 0; 1]));

% leg 2 and 3: only pin position (H01) are different
% 2: in (+x, -y, 0) quadrant
theta_2_1 = -pi/3;
d_2_1 = 0;
a_2_1 = pin_center_dist;
alpha_2_1 = pi/2;
H_2_01 = [cos(theta_2_1), -sin(theta_2_1) * cos(alpha_2_1), sin(theta_2_1) * sin(alpha_2_1), a_2_1 * cos(theta_2_1);
         sin(theta_2_1), cos(theta_2_1) * cos(alpha_2_1), -cos(theta_2_1) * sin(alpha_2_1), a_2_1 * sin(theta_2_1);
         0, sin(alpha_2_1), cos(alpha_2_1), d_2_1;
         0, 0, 0, 1];
% HTM from earth frame to link frames
H_2_02 = vpa(simplify(H_2_01 * H_1_12));
H_2_02 = mapSymType(H_2_02, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
H_2_03 = vpa(simplify(H_2_02 * H_1_231));
H_2_03 = mapSymType(H_2_03, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
H_2_04 = vpa(simplify(H_2_03 * H_1_3132 * H_1_3233 * H_1_334));
H_2_04 = mapSymType(H_2_04, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
p_pin_2 = H_2_01 * [0; 0; 0; 1];
p_motor_2 = vpa(simplify(H_2_02 * [0; 0; 0; 1]));
p_ball_2 = vpa(simplify(H_2_03 * [0; 0; 0; 1])); 
p_center_2 = vpa(simplify(H_2_04 * [0; 0; 0; 1]));

% 3: in (-x, 0, 0)
theta_3_1 = pi;
d_3_1 = 0;
a_3_1 = pin_center_dist;
alpha_3_1 = pi/2;
H_3_01 = [cos(theta_3_1), -sin(theta_3_1) * cos(alpha_3_1), sin(theta_3_1) * sin(alpha_3_1), a_3_1 * cos(theta_3_1);
         sin(theta_3_1), cos(theta_3_1) * cos(alpha_3_1), -cos(theta_3_1) * sin(alpha_3_1), a_3_1 * sin(theta_3_1);
         0, sin(alpha_3_1), cos(alpha_3_1), d_3_1;
         0, 0, 0, 1];
% HTM from earth frame to link frames
H_3_02 = vpa(simplify(H_3_01 * H_1_12));
H_3_02 = mapSymType(H_3_02, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
H_3_03 = vpa(simplify(H_3_02 * H_1_231));
H_3_03 = mapSymType(H_3_03, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
H_3_04 = vpa(simplify(H_3_03 * H_1_3132 * H_1_3233 * H_1_334));
H_3_04 = mapSymType(H_3_04, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
p_pin_3 = H_3_01 * [0; 0; 0; 1];
p_motor_3 = vpa(simplify(H_3_02* [0; 0; 0; 1]));
p_ball_3 = vpa(simplify(H_3_03 * [0; 0; 0; 1])); 
p_center_3 = vpa(simplify(H_3_04 * [0; 0; 0; 1]));

% cleaning (dropping terms with very small coefficient, which appear due to rounding errors)
% p_center_1 = mapSymType(p_center_1, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x))
% p_center_2 = mapSymType(p_center_2, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));
% p_center_3 = mapSymType(p_center_3, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x));

%% configuration at horizontal position
% calculate positions
% 1
tp_1_val = 0;
dm_1_val = 1;
tbx_1_val = 0;
tby_1_val = 0;
tbz_1_val = 0;
p_pin_1_val = subs(p_pin_1, [tp, dm, tbx, tby, tbz], [tp_1_val, dm_1_val, tbx_1_val, tby_1_val, tbz_1_val]);
p_motor_1_val = subs(p_motor_1, [tp, dm, tbx, tby, tbz], [tp_1_val, dm_1_val, tbx_1_val, tby_1_val, tbz_1_val]);
p_ball_1_val = subs(p_ball_1, [tp, dm, tbx, tby, tbz], [tp_1_val, dm_1_val, tbx_1_val, tby_1_val, tbz_1_val]);
p_center_1_val = subs(p_center_1, [tp, dm, tbx, tby, tbz], [tp_1_val, dm_1_val, tbx_1_val, tby_1_val, tbz_1_val]);
% 2
tp_2_val = 0;
dm_2_val = 1;
tbx_2_val = 0;
tby_2_val = pi/2;
tbz_2_val = 0;
p_pin_2_val = subs(p_pin_2, [tp, dm, tbx, tby, tbz], [tp_2_val, dm_2_val, tbx_2_val, tby_2_val, tbz_2_val]);
p_motor_2_val = subs(p_motor_2, [tp, dm, tbx, tby, tbz], [tp_2_val, dm_2_val, tbx_2_val, tby_2_val, tbz_2_val]);
p_ball_2_val = subs(p_ball_2, [tp, dm, tbx, tby, tbz], [tp_2_val, dm_2_val, tbx_2_val, tby_2_val, tbz_2_val]);
p_center_2_val = subs(p_center_2, [tp, dm, tbx, tby, tbz], [tp_2_val, dm_2_val, tbx_2_val, tby_2_val, tbz_2_val]);
% 3
tp_3_val = 0;
dm_3_val = 1;
tbx_3_val = 0;
tby_3_val = pi/2;
tbz_3_val = 0;
p_pin_3_val = subs(p_pin_3, [tp, dm, tbx, tby, tbz], [tp_3_val, dm_3_val, tbx_3_val, tby_3_val, tbz_3_val]);
p_motor_3_val = subs(p_motor_3, [tp, dm, tbx, tby, tbz], [tp_3_val, dm_3_val, tbx_3_val, tby_3_val, tbz_3_val]);
p_ball_3_val = subs(p_ball_3, [tp, dm, tbx, tby, tbz], [tp_3_val, dm_3_val, tbx_3_val, tby_3_val, tbz_3_val]);
p_center_3_val = subs(p_center_3, [tp, dm, tbx, tby, tbz], [tp_3_val, dm_3_val, tbx_3_val, tby_3_val, tbz_3_val]);

%% plotting
% plotting
% 1
P_1 = [p_pin_1_val, p_motor_1_val, p_ball_1_val, p_center_1_val];
figure
hold on
plot3([0, p_pin_1_val(1)], [0, p_pin_1_val(2)], [0, p_pin_1_val(3)], 'r:', 'LineWidth', 3)
plot3(P_1(1, :), P_1(2, :), P_1(3, :), 'k', 'LineWidth', 3)
% 2
P_2 = [p_pin_2_val, p_motor_2_val, p_ball_2_val, p_center_2_val];
plot3([0, p_pin_2_val(1)], [0, p_pin_2_val(2)], [0, p_pin_2_val(3)], 'g:', 'LineWidth', 3)
plot3(P_2(1, :), P_2(2, :), P_2(3, :), 'k', 'LineWidth', 3)
% 3
P_3 = [p_pin_3_val, p_motor_3_val, p_ball_3_val, p_center_3_val];
plot3([0, p_pin_3_val(1)], [0, p_pin_3_val(2)], [0, p_pin_3_val(3)], 'b:', 'LineWidth', 3)
plot3(P_3(1, :), P_3(2, :), P_3(3, :), 'k', 'LineWidth', 3)
%% Solve IK problem
s1 = [0; 0; 0; 0; 0];
s2 = [0; 0; 0; 0; 0];
s3 = [0; 0; 0; 0; 0];

% inverse kinematics
% input: platform configuration (2 theta, 1 center height (the other two coordinates are fixed))
theta_pitch = 0;
theta_roll = pi/6;
z_center = 15;
% find vectors from center to each ball joint
% rotation matrix from plane frame to world frame
% first pitch, then roll
R_pitch = [1, 0, 0;
           0, cos(theta_pitch), -sin(theta_pitch);
           0, sin(theta_pitch), cos(theta_pitch)];
R_roll = [cos(theta_roll), 0, -sin(theta_roll);
          0, 1, 0;
          sin(theta_roll), 0, cos(theta_roll)];
R_tot = R_pitch * R_roll;
% vectors in platform frame
vec1 = [mount_center_dist * cos(pi/3); mount_center_dist * sin(pi/3); 0];
vec2 = [mount_center_dist * cos(2 * pi/3); mount_center_dist * sin(2 * pi/3); 0];
vec3 = [-mount_center_dist; 0; 0];
% transformed vectors
vec1_tr = R_tot * vec1;
vec2_tr = R_tot * vec2;
vec3_tr = R_tot * vec3;
% vec2_tr(3) = -vec2_tr(3);
% IK equations
% 1
% collect equations (center position and vec components)
% vector based on platform angle
vec1_ik = p_ball_1 - p_center_1;
% use x and z components (for leg 3, y component is always 0)
eqs_1 = [p_center_1(1:3, :); vec1_ik(1); vec1_ik(2); vec1_ik(3)];
leg1 = @(x) double(subs(eqs_1, [tp, dm, tbx, tby, tbz], [x(1), x(2), x(3), x(4), x(5)])) - [0; 0; z_center; vec1_tr(1); vec1_tr(2); vec1_tr(3)];
% initial guess
x0 = [0; 0; 0; 0; 0];
% x0 = [0; z_center - motor_retracted; 0; 0; 0];
% solve equations
s1 = fsolve(leg1, x0);
%2
% vector based on platform angle
vec2_ik = p_ball_2 - p_center_2;
% use x and z components (for leg 3, y component is always 0)
eqs_2 = [p_center_2(1:3, :); vec2_ik(1); vec2_ik(2); vec2_ik(3)];
leg2 = @(x) double(subs(eqs_2, [tp, dm, tbx, tby, tbz], [x(1), x(2), x(3), x(4), x(5)])) - [0; 0; z_center; vec2_tr(1); vec2_tr(2); vec2_tr(3)];
% solve equations
s2 = fsolve(leg2, x0);
%3
% vector based on platform angle
vec3_ik = p_ball_3 - p_center_3;
% use x and z components (for leg 3, y component is always 0)
eqs_3 = [p_center_3(1:3, :); vec3_ik(1); vec3_ik(2); vec3_ik(3)];
leg3 = @(x) double(subs(eqs_3, [tp, dm, tbx, tby, tbz], [x(1), x(2), x(3), x(4), x(5)])) - [0; 0; z_center; vec3_tr(1); vec3_tr(2); vec3_tr(3)];
% solve equations
s3 = fsolve(leg3, x0);

%% calculate IK solution positions
p_pin_1_val = double(subs(p_pin_1, [tp; dm; tbx; tby; tbz], s1));
p_motor_1_val = double(subs(p_motor_1, [tp; dm; tbx; tby; tbz], s1));
p_ball_1_val = double(subs(p_ball_1, [tp; dm; tbx; tby; tbz], s1));
p_center_1_val = double(subs(p_center_1, [tp; dm; tbx; tby; tbz], s1));

p_pin_2_val = double(subs(p_pin_2, [tp; dm; tbx; tby; tbz], s2));
p_motor_2_val = double(subs(p_motor_2, [tp; dm; tbx; tby; tbz], s2));
p_ball_2_val = double(subs(p_ball_2, [tp; dm; tbx; tby; tbz], s2));
p_center_2_val = double(subs(p_center_2, [tp; dm; tbx; tby; tbz], s2));

p_pin_3_val = double(subs(p_pin_3, [tp; dm; tbx; tby; tbz], s3));
p_motor_3_val = double(subs(p_motor_3, [tp; dm; tbx; tby; tbz], s3));
p_ball_3_val = double(subs(p_ball_3, [tp; dm; tbx; tby; tbz], s3));
p_center_3_val = double(subs(p_center_3, [tp; dm; tbx; tby; tbz], s3));

     