% solve for platform configuration from each link length
% fully specified when the angle of the pin joints are known
% prev_pos: [th1_guess; th2_guess; th3_guess], initial guess based on FK
% solution in the previous time step (should be close to the solution in the current time step)
function [th1, th2, th3] = RPS_forward_kinematics(L1, L2, L3, h, g, prev_pos)
% linkage parameters
rho = h / g;
% if prev_pos(3) == 0 
%     prev_pos(3) = 0.01;
% end
% if prev_pos(2) == 0 
%     prev_pos(2) = 0.01;
% end
% if prev_pos(1) == 0 
%     prev_pos(1) = 0.01;
% end
L1 = L1 / g;
L2 = L2 / g;
L3 = L3 / g;
FK_eqs = @(t) [L1^2 + L2^2 + 3 - 3 * rho^2 + L1 * L2 * cos(t(1)) * cos(t(2)) - 2 * L1 * L2 * sin(t(1)) * sin(t(2)) - 3 * L1 * cos(t(1)) - 3 * L2 * cos(t(2));
               L2^2 + L3^2 + 3 - 3 * rho^2 + L2 * L3 * cos(t(2)) * cos(t(3)) - 2 * L2 * L3 * sin(t(2)) * sin(t(3)) - 3 * L2 * cos(t(2)) - 3 * L3 * cos(t(3));
               L3^2 + L1^2 + 3 - 3 * rho^2 + L3 * L1 * cos(t(3)) * cos(t(1)) - 2 * L3 * L1 * sin(t(3)) * sin(t(1)) - 3 * L3 * cos(t(3)) - 3 * L1 * cos(t(1))];
% use newton's method to solve for pin angles
% define the objective function
sol = fsolve(FK_eqs, prev_pos);
th1 = sol(1);
th2 = sol(2);
th3 = sol(3);
end