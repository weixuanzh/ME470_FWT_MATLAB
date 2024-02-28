% plot the configuration of the platform in the given configuration
% variables to uniquely define configuration: 
% L: actuator length
% theta: pin joint angles
% h: distance between ball joints and center (top plate)
% g: distance between pins and center (bottom plate)
% need to call figure and hold on before using this function
function [z_center] = RPS_plotting(L1, L2, L3, theta1, theta2, theta3, h, g, view_point)

% base pin joint coordinates
p1 = [g; 0; 0];
p2 = [-0.5 * g; sqrt(3) * g / 2; 0];
p3 = [-0.5 * g; -sqrt(3) * g / 2; 0];

% top ball joint coordinates
b1 = p1 + [-L1 * cos(theta1); 0; L1 * sin(theta1)];
b2 = p2 + [L2 * cos(theta2) * 0.5; -L2 * cos(theta2) * sqrt(3) / 2; L2 * sin(theta2)];
b3 = p3 + [L3 * cos(theta3) * 0.5; L3 * cos(theta3) * sqrt(3) / 2; L3 * sin(theta3)];
z_center = (b1(3) + b2(3) + b3(3))/3;

% plot base plate
plot3([p1(1), p2(1); p2(1), p3(1); p3(1), p1(1)], [p1(2), p2(2); p2(2), p3(2); p3(2), p1(2)], [p1(3), p2(3); p2(3), p3(3); p3(3), p1(3)] ,'b')
hold on
% label pin locations
text(p1(1), p1(2), p1(3), 'pin1')
text(p2(1), p2(2), p2(3), 'pin2')
text(p3(1), p3(2), p3(3), 'pin3')

% plot top plate
plot3([b1(1), b2(1); b2(1), b3(1); b3(1), b1(1)], [b1(2), b2(2); b2(2), b3(2); b3(2), b1(2)], [b1(3), b2(3); b2(3), b3(3); b3(3), b1(3)], 'b')
% plot linkages
plot3([p1(1), b1(1)], [p1(2), b1(2)], [p1(3), b1(3)], 'r')
plot3([p2(1), b2(1)], [p2(2), b2(2)], [p2(3), b2(3)], 'r')
plot3([p3(1), b3(1)], [p3(2), b3(2)], [p3(3), b3(3)], 'r')
% plot center
plot3((b1(1) + b2(1) + b3(1))/3, (b1(2) + b2(2) + b3(2))/3, z_center, 'o')
xlim([-200, 200])
ylim([-200, 200])
zlim([-50, 500])
% side view
if view_point == 0
    view(0, 0)
end
if view_point == 1
    view(90, 0)
end
if view_point == 2
    view(45, 45)
end
hold off
end
