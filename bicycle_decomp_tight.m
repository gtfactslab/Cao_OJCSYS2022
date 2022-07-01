clear
clc

%% PARAMS
global lr lf
lr = 5.5 * 0.6;
lf = 5.5 * 0.4;

% INPUTS
a = -10;
delta_f = 0.3;
T_step = 0.005;
T = 0:T_step:1;

% INITIAL STATE
bike_init = [0;0;0;20];%[0;0;0;20];

%% SANITY CHECK
options = optimoptions('quadprog','Display','None');

% Sanity Check, two copies of same inputs should lead to same outputs
% Original Dynamics
[tso,os]=ode45(@(t,o) bicycle_update(o,u(t)),T,bike_init,options);

% Embedding System
[tse,es]=ode45(@(t,e) bicycle_embed(e(1:4),u(t),e(5:8),u(t)),T,[bike_init; bike_init],options); 

% Plot
plot_states(1, tso, os, tse, es)

%% STATE UNCERTAINTY
% now we have state uncertainty
bike_init_uncertainty = 5*[0.1; 0.1; 0.01; 0];
bike_init_rect = [bike_init - bike_init_uncertainty;
                  bike_init + bike_init_uncertainty];
input_uncertainty = [0; 0];%[0.1; 0.01];


%% FIT PARAMS FOR DISTURBANCE
global f_bar_star cov_f_star a_space a_step
global sigma_f l sigma_n
sigma_f = 45;
l = 11;
sigma_n = 0.1; %0.1
a_step = 0.02;
a_space = -20:a_step:10;

global known_observations
starting_u = u(0);
starting_a = starting_u(1);
known_observations = [starting_a, disturbance(starting_a)];
start_time = 0;



% Actual Behavior
end_time = 1.6;
T = 0:T_step:end_time;
full_T = 0:T_step:end_time;
[tso,os]=ode45(@(t,o) bicycle_update_dist(o,u(t)),full_T,bike_init,options);

% Original Dynamics Without Disturbance
[tsw,ws]=ode45(@(t,w) bicycle_update(w,u(t)),full_T,bike_init,options);

% Plot inputs for debugging
u_vec = [];
for t = full_T
    u_vec = [u_vec, u(t)];
end
figure(9)
subplot(2, 1, 1)
plot(full_T, u_vec(1, :))
xlabel('time (sec)')
ylabel('acceleration (m/s^2)')
title('Input Acceleration')
subplot(2, 1, 2)
plot(full_T, u_vec(2, :));
xlabel('time (sec)')
ylabel('angle (rad)')
title('Input Steering Angle')
axis([0, 1.6, -0.7, 0.7])

for i = 1:50
    now = tic();
    [f_bar_star, cov_f_star] =  fit_params(known_observations(:, 1), known_observations(:, 2), a_space, sigma_f, l, sigma_n);
    GP_time = toc(now)

    % Now with same initial state and input, actual behavior is way different
    % Actual Behavior
    %[tsd,ds]=ode45(@(t,d) bicycle_update_dist(d,u(t)),T,bike_init,options);
    now = tic();
    % Embedding System with Disturbance
    [tsed,eds]=ode45(@(t,ed) bicycle_embed_dist(ed(1:4),u(t)-input_uncertainty,ed(5:8),u(t)+input_uncertainty),T,bike_init_rect,options);
    reachset_time = toc(now)
    
    % Original Dynamics Without Disturbance (update with new start)
    %[tsw,ws]=ode45(@(t,w) bicycle_update(w,u(t)),T,bike_init,options);
    
    save_plots = [0, 0, 0, 0];
    
    time_to_finish = GP_time + reachset_time
    
    plot_states(4, tso, os, tsed, eds)
    filename = ['figures/full_states_', num2str(i), '.tex'];
    if save_plots(1)
        matlab2tikz('filename', filename);
    end
    plot_trajectories(5, os, eds)
    legend('True Trajectory', 'Lower Bound Approximation', 'Upper Bound Approximation');
    title('Reachable Set Approximation')
    filename = ['figures/full_traj_', num2str(i), '.tex'];
    if save_plots(2)
        matlab2tikz('filename', filename);
    end
    plot_GP(6)
    filename = ['figures/biginit_GP_', num2str(i), '.tex'];
    if save_plots(3)
        matlab2tikz('filename', filename);
    end
    
    
    compare_trajs(7, os, ws)
    filename = ['figures/compare_', num2str(i), '.tex'];
    if save_plots(4)
        matlab2tikz('filename', filename);
    end
    
%     border = calculate_reachset_known_u(bike_init_rect, @u, T, true);
%     figure(5)
%     hold on
%     plot(border(:, 1), border(:, 2), 'm-');
%     hold off
%     matlab2tikz('filename', 'figures/full_with_reachset_biggerinit_4.tex')
    
    start_time = start_time + 0.1;
    T = start_time:T_step:end_time;
    refresh_time =  ceil(start_time/T_step);
    bike_init = os(refresh_time, :)';
    bike_init_rect = [bike_init - bike_init_uncertainty;
                  bike_init + bike_init_uncertainty];
    starting_u = u(start_time);
    a_obs = starting_u(1);
    known_observations = [known_observations; a_obs, disturbance(a_obs)];
%     
%     if i > 15
%         u = [2; 0];
%     end
    
w = waitforbuttonpress;
end
%% SIM FUNCTIONS
function input = u(t)
    angle_mag = 0.5;
    if t < 0.3
        input = [-40*t - 2; angle_mag*t/0.3]; 
    elseif t < 0.5
        input = [-40*t - 2; angle_mag];
    elseif t < 0.7
        input = [-22 + 20*(t - 0.5)/0.2; angle_mag - angle_mag*(t - 0.5)/0.2];
    elseif t < 0.9
        input = [-2; -angle_mag*(t - 0.7)/0.2];
    elseif t < 1.4
        input = [-2; -angle_mag];
    elseif t < 1.6
        input = [-2; -angle_mag + angle_mag*(t - 1.4)/0.2];
    else
        input = [-2; 0];
    end
    input = input .* [1; -1];
end

%% PLOTTING
function res = plot_states(fig, tso, os, tse, es)
    figure(fig)
    subplot(2, 2, 1)
    plot(tso, os(:, 1), 'k--')
    hold on
    plot(tse, es(:, 1), 'r-')
    plot(tse, es(:, 5), 'b-')
    hold off
    axis([0, 2.5, 0, 35])
    title("X Position")
    xlabel("t")
    ylabel("X")
    subplot(2, 2, 2)
    plot(tso, os(:, 2), 'k--')
    hold on
    plot(tse, es(:, 2), 'r-')
    plot(tse, es(:, 6), 'b-')
    hold off
    axis([0, 2.5, -40, 0])
    title("Y Position")
    xlabel("t")
    ylabel("Y")
    subplot(2, 2, 3)
    plot(tso, os(:, 3), 'k--')
    hold on
    plot(tse, es(:, 3), 'r-')
    plot(tse, es(:, 7), 'b-')
    hold off
    axis([0, 2.5, -1, 0])
    title("Angle")
    xlabel("t")
    ylabel("\psi")
    subplot(2, 2, 4)
    plot(tso, os(:, 4), 'k--')
    hold on
    plot(tse, es(:, 4), 'r-')
    plot(tse, es(:, 8), 'b-')
    hold off
    axis([0, 2.5, 16, 21])
    title("Velocity")
    xlabel("t")
    ylabel("v")
end

function res = plot_trajectories(fig, os, es)
    figure(fig)
    plot(os(:, 1), os(:, 2), 'k--')
    hold on
    plot(es(:, 1), es(:, 2), 'r-')
    plot(es(:, 5), es(:, 6), 'b-')
    corner = [min(es(1, 1), es(1, 5)), min(es(1, 2), es(1, 6))];
    sizes = [abs(es(1,5) - es(1, 1)), abs(es(1, 6) - es(1, 2))];
    rectangle('Position',[corner, sizes]);
    corner = [min(es(end, 1), es(end, 5)), min(es(end, 2), es(end, 6))];
    sizes = [abs(es(end,5) - es(end, 1)), abs(es(end, 6) - es(end, 2))];
    rectangle('Position',[corner, sizes]);
    axis equal
    hold off
end

function res = compare_trajs(fig, os, es)
    figure(fig)
    plot(os(:, 1), os(:, 2), 'k-')
    hold on
    plot(es(:, 1), es(:, 2), 'r--')
    axis equal
    hold off
end


function border = calculate_reachset(init_rect, u_range, T, dist_present)
    endpoints = [];
    step = 0.01;
    options = optimoptions('quadprog','Display','None');
    for x = init_rect(1):step:init_rect(5)
        x
        for y = init_rect(2):step:init_rect(6)
            for psi = init_rect(3):step:init_rect(7)
                for v = init_rect(4):step:init_rect(8)
                    for a = u_range(1):step:u_range(3)
                        for df = u_range(2):step:u_range(4)
                            if ~dist_present
                                [tso,os]=ode45(@(t,o) bicycle_update(o,[a;df]),T,[x;y;psi;v],options);
                            else
                                [tso,os]=ode45(@(t,o) bicycle_update_dist(o,[a;df]),T,[x;y;psi;v],options);
                            end
                            endpoints = [endpoints; os(end, 1:2)];
                        end
                    end
                end
            end
        end
    end
    [k, av] = convhull(endpoints);
    border = endpoints(k, :);
end

function border = calculate_reachset_known_u(init_rect, u_fn, T, dist_present)
    endpoints = [];
    step = 0.01;
    options = optimoptions('quadprog','Display','None');
    for x = init_rect(1):step:init_rect(5)
        x
        for y = init_rect(2):step:init_rect(6)
            for psi = init_rect(3):step:init_rect(7)
                for v = init_rect(4):step:init_rect(8)
                    if ~dist_present
                        [tso,os]=ode45(@(t,o) bicycle_update(o,u_fn(t)),T,[x;y;psi;v],options);
                    else
                        [tso,os]=ode45(@(t,o) bicycle_update_dist(o,u_fn(t)),T,[x;y;psi;v],options);
                    end
                    endpoints = [endpoints; os(end, 1:2)];
                end
            end
        end
    end
    [k, av] = convhull(endpoints);
    border = endpoints(k, :);
end

function res = plot_GP(fig)
    global a_space f_bar_star cov_f_star known_observations
    sigma_confidence = 3;
    figure(fig)
    plot(a_space, disturbance(a_space), 'k-'); % actual
    hold on
    scatter(known_observations(:, 1), known_observations(:, 2), 'ko'); % observations
    plot(a_space, f_bar_star, 'b--'); % mean
    std_f_star = sqrt(diag(cov_f_star));
    upper_x = (f_bar_star + sigma_confidence * std_f_star)';
    lower_x = (f_bar_star - sigma_confidence * std_f_star)';
    fill_Ax = [a_space, fliplr(a_space)];
    fill_Bx = [lower_x, fliplr(upper_x)];
    fill(fill_Ax, fill_Bx, 'k', 'facealpha', 0.2, 'edgealpha', 0);
    title('a-dependent Disturbance')
    legend('Actual', 'Observations', 'Estimated Mean', 'Confidence Bound');
    hold off
end

%% DECOMPOSITION
function out = beta(delta_f)
    global lr lf
    out = atan(lr*tan(delta_f)/(lr+lf));
end

function out = d_w1w2(w, what)
    % assuming 2d input
    values = [w(1)*w(2),
              w(1)*what(2),
              what(1)*what(2),
              what(1)*w(2)];
    if all(w <= what)
        out = min(values);
    elseif all(what <= w)
        out = max(values);
    else
        error("w (%s) and what (%s) are not component-wise comparable", mat2str(w), mat2str(what))
    end
end

function out = d(x, u, xhat, uhat)
    global lr lf
    % x = [X, Y, psi, v]
    % u = [a, delta_f]
    dX = d_w1w2([x(4), d_cos(x(3) + beta(u(2)), xhat(3) + beta(uhat(2)))], [xhat(4), d_cos(xhat(3) + beta(uhat(2)), x(3) + beta(u(2)))]);
    dY = d_w1w2([x(4), d_sin(x(3) + beta(u(2)), xhat(3) + beta(uhat(2)))], [xhat(4), d_sin(xhat(3) + beta(uhat(2)), x(3) + beta(u(2)))]);
    dpsi = (1/lr)*d_w1w2([x(4), d_sin(beta(u(2)), beta(uhat(2)))], [xhat(4), d_sin(beta(uhat(2)), beta(u(2)))]);
    dv = u(1);
    
    out = [dX; dY; dpsi; dv];
end

function x_dot = bicycle_embed(x, u, xhat, uhat)
    x_dot = [d(x, u, xhat, uhat); d(xhat, uhat, x, u)];
end

function x_dot = bicycle_update(x, u)
    global lr lf
    % x = [X, Y, psi, v]
    % u = [a, delta_f]
    psi = x(3);
    v = x(4);
    a = u(1);
    delta_f = u(2);
    
    x_dot = [ v * cos(psi + beta(delta_f));
              v * sin(psi + beta(delta_f));
              (v/lr)*sin(beta(delta_f));
              a];
end

function x_dot = bicycle_embed_dist(x, u, xhat, uhat)
    global a_step
    x_dot = [d(x, u, xhat, uhat); d(xhat, uhat, x, u)];
    a = u(1);
    ahat = uhat(1);
    [wmin, wmax] = disturbance_bounds_frozen(a, ahat);
%     [wmin, ~] = disturbance_bounds(a - a_step/2, a  + a_step/2);
%     [~, wmax] = disturbance_bounds(ahat - a_step/2, ahat  + a_step/2);
    x_dot(4) = x_dot(4) + wmin;
    x_dot(8) = x_dot(8) + wmax;
end

function x_dot = bicycle_update_dist(x, u)
    global lr lf
    % x = [X, Y, psi, v]
    % u = [a, delta_f]
    psi = x(3);
    v = x(4);
    a = u(1);
    delta_f = u(2);
    
    x_dot = [ v * cos(psi + beta(delta_f));
              v * sin(psi + beta(delta_f));
              (v/lr)*sin(beta(delta_f));
              a + disturbance(a)];
end

%% DISTURBANCE MODELING
function disturbance = disturbance(x)
    disturbance = -0.8.*x';
end

function [wmin, wmax] = disturbance_bounds(a, ahat)
    sigma_confidence = 3;
    global f_bar_star cov_f_star a_space
    
    range_min = min(a, ahat);
    range_max = max(a, ahat);
    if (range_min < min(a_space) || range_max > max(a_space))
        error('Attempting to measure disturbance outside of known space, expand range');
    end
    
    % get min/max functions
    
    std_f_star = sqrt(diag(cov_f_star));
    min_func = f_bar_star - (sigma_confidence * std_f_star);
    max_func = f_bar_star + (sigma_confidence * std_f_star);
    
    % isolate to range we care about
        
    min_func = min_func(a_space >= range_min & a_space <= range_max);
    max_func = max_func(a_space >= range_min & a_space <= range_max);

    wmin = min(min_func);
    wmax = max(max_func);
end

function [wmin, wmax] = disturbance_bounds_frozen(a, ahat)
    sigma_confidence = 3;
    global known_observations sigma_f l sigma_n
    [fbs, cfs] = fit_params(known_observations(:, 1), known_observations(:, 2), [a, ahat], sigma_f, l, sigma_n);
    sfs = sqrt(diag(cfs));
    wmin = fbs(1) - sigma_confidence * sfs(1);
    wmax = fbs(2) + sigma_confidence * sfs(2);
end

%% GAUSSIAN PROCESS FUNCTIONS
function kernel =  kernel(x, y, sigma_f, l)
    % squared exponential aka radial basis kernel
    kernel = sigma_f * exp(-1*(x-y)^2 / (2*l^2));
end

function [K, K_star2, K_star] = cov_matrices(x, x_star, sigma_f, l)
    x_size = numel(x);
    xs_size = numel(x_star);
    K = zeros(x_size);
    K_star2 = zeros(xs_size);
    K_star = zeros([xs_size, x_size]);
    
    for i = 1:x_size
        for j = 1:x_size
            K(i, j) = kernel(x(i), x(j), sigma_f, l);
        end
    end
    
    for i = 1:xs_size
        for j = 1:xs_size
            K_star2(i, j) = kernel(x_star(i), x_star(j), sigma_f, l);
        end
    end
    
    for i = 1:xs_size
        for j = 1:x_size
            K_star(i, j) = kernel(x_star(i), x(j), sigma_f, l);
        end
    end
    
end

function [f_bar_star, cov_f_star] = gpr_params(y, K, K_star2, K_star, sigma_n)
    n = size(K);
    n = n(1);
    
    % calculate cholesky decomp
    L = chol(K + ((sigma_n^2) * eye(n)));
    alpha = L \ (L' \ y);

    % mean
    f_bar_star = K_star * alpha;
    
    %covariance
    v = (L' \ K_star')';
    cov_f_star = K_star2 - (v * v');
   
end

function [f_bar_star, cov_f_star] = fit_params(x, y, x_star, sigma_f, l, sigma_n)
    [K, K_star2, K_star] = cov_matrices(x, x_star, sigma_f, l);
    [f_bar_star, cov_f_star] = gpr_params(y, K, K_star2, K_star, sigma_n);
end
