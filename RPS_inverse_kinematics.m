% solve for each link length given euler angles and center height
% h: distance between ball joints and center (top plate)
% g: distance between pins and center (bottom plate)
function [d1, d2, d3] = RPS_inverse_kinematics(pz, alpha, beta, h, g)
% dependent variables
px = -0.5 * h * (1 - cos(beta)) * cos(2 * alpha);
py = 0.5 * h * (1 - cos(beta)) * sin(2 * alpha);

% total length of actuator
d1 = g^2 + h^2 + px^2 + py^2 + pz^2 - 2 * g * px + 2 * h * ((cos(alpha))^2 * cos(beta) + (sin(alpha))^2) * (px - g) + h * (cos(beta) - 1) * sin(2 * alpha) * py - 2 * h * sin(beta) * cos(alpha) * pz;
d1 = sqrt(d1);

d2 = g^2 + h^2 + px^2 + py^2 + pz^2 + g * px - sqrt(3) * g * py - h * ((cos(alpha))^2 * cos(beta) + (sin(alpha))^2 - sqrt(3) * cos(alpha) * sin(alpha) * (cos(beta) - 1)) * (px + 0.5 * g) - h * (sin(alpha) * cos(alpha) * (cos(beta) - 1) - sqrt(3) * ((sin(alpha))^2 * cos(beta) + (cos(alpha))^2)) * (py - sqrt(3) * 0.5 * g) + h * sin(beta) * (cos(alpha) - sqrt(3) * sin(alpha)) * pz;
d2 = sqrt(d2);

d3 = g^2 + h^2 + px^2 + py^2 + pz^2 + g * px + sqrt(3) * g * py - h * ((cos(alpha))^2 * cos(beta) + (sin(alpha))^2 - sqrt(3) * cos(alpha) * sin(alpha) * (cos(beta) - 1)) * (px + 0.5 * g) - h * (sin(alpha) * cos(alpha) * (cos(beta) - 1) + sqrt(3) * ((sin(alpha))^2 * cos(beta) + (cos(alpha))^2)) * (py + sqrt(3) * 0.5 * g) + h * sin(beta) * (cos(alpha) + sqrt(3) * sin(alpha)) * pz;
d3 = sqrt(d3);
end