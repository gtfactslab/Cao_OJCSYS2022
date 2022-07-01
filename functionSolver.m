clear;

% params
axis_range = [-10, 10, -10, 10];
using_GP = false;
slope = 2; % 1 for part 2
gamma = @(x_eq)gammaLinear(x_eq, slope);

% begin execution
x0 = [-7, -7, 7, 7]; % part 1
% x0 =
% [-1.72405339857778,-3.40045286756293,1.72405339857778,3.40045286756293]; % part 2
x_eq = x0;
x_prev = [0, 0, 0, 0];

diff = sum(abs(x_eq - x_prev));
iterations = 0;
history = [x_eq];
complete = false;

figure(1)
clf(1)
if using_GP
    plot_GP_surfaces;
else
    plot_gamma_surfaces;
end
while diff > .00001
    figure(2)
    clf(2)
    plot_rectangle_iterations;
    % get max/min gamma
    [gamma_min, gamma_max] = gamma(x_eq);
    

    W = [gamma_min, gamma_max];
    %W = [-2, 2];
    func = @(x)e2(x, W);
    x_prev = x_eq;
    x_eq = fsolve(func, x_eq); 
    
    iterations = iterations + 1;
    history = [history; x_eq];
    diff = sum(abs(x_eq - x_prev));
    
    
    figure(1)
    clf(1)
    if using_GP
        plot_GP_surfaces;
    else
        plot_gamma_surfaces;
    end
end
complete = true;
figure(1)
clf(1)

plot_gamma_surfaces;

figure(2)
clf(2)
plot_rectangle_iterations;