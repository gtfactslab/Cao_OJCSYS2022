function [gamma_min, gamma_max] = gammaLinear(x_eq, slope)
% x_eq = [x1, x2, x1_hat, x2_hat]
x_under = x_eq(1);
x_over = x_eq(3);
if (x_under <= 0) && (x_over <= 0)
    gamma_min = slope * x_under;
    gamma_max = -slope * x_under;
elseif (x_under <= 0) && (x_over > 0)
    gamma_min = min([slope * x_under, -slope * x_over]);
    gamma_max = max([-slope * x_under, slope * x_over]);
else
    gamma_min = -slope * x_under;
    gamma_max = slope * x_under;
end