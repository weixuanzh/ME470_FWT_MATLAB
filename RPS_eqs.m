function [res] = RPS_eqs(t, L1, L2, L3, h, g)

rho = h / g;

res = [L1^2 + L2^2 + 3 - 3 * rho^2 + L1 * L2 * cos(t(1)) * cos(t(2)) - 2 * L1 * L2 * sin(t(1)) * sin(t(2)) - 3 * L1 * cos(t(1)) - 3 * L2 * cos(t(2));
       L2^2 + L3^2 + 3 - 3 * rho^2 + L2 * L3 * cos(t(2)) * cos(t(3)) - 2 * L2 * L3 * sin(t(2)) * sin(t(3)) - 3 * L2 * cos(t(2)) - 3 * L3 * cos(t(3));
       L3^2 + L1^2 + 3 - 3 * rho^2 + L3 * L1 * cos(t(3)) * cos(t(1)) - 2 * L3 * L1 * sin(t(3)) * sin(t(1)) - 2 * L3 * cos(t(3)) - 3 * L1 * cos(t(1))];
end