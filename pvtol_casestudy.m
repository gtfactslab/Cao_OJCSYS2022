% simulation params
global debug noisy_obs 
debug = false;
noisy_obs = true;
global m g m_act g_act actuator_noise%%%Noise not implemented now
m=1; %mass used for control computation
g=9.8; %gravity used for control computation
actuator_noise=0;
sensor_noise=0;
parameter_mismatch=0;
m_act= m+parameter_mismatch*randn; %mass used for state update
g_act=g+parameter_mismatch*randn; %gravity used for state update
global sigma_n assumed_sigma_n;
sigma_n = 0.1; % 0.01;training sample error std. dev.
assumed_sigma_n = 0.1;
sigma_f = 60; % 30; kernel function amplitude
l = 8;%5; % kernel function locality;
global sigma_confidence
sigma_confidence = 2;
global K1 K2 K3 %[-100, -68, -17]
K1=-100;
K2=-68;
K3=-17;
global to_observe finv_rect finv_buffer obj_buffer
obj_buffer = 0.2; % range of obj within which observation is taken
finv_buffer = 0.18; % buffer from finv set where control action begins to revert to safety
%rng(0);
global safety_AX safety_BX safety_AY safety_BY

% initialize needed variables
observations_X = [];
observations_Y = [];
x_space = -50:0.025:50;
y_space = -50:0.025:50;
plot_cutoff = 10;
[sampling_points_X, sampling_points_Y] = meshgrid(-plot_cutoff:plot_cutoff, -plot_cutoff:plot_cutoff);
sampling_points_X = sampling_points_X(:);
sampling_points_Y = sampling_points_Y(:);
sampling_points_U = disturbance_x(sampling_points_X);
sampling_points_V = disturbance_y(sampling_points_Y);

% each GP starts with a single observation at 0 (maybe plus a few near 0?)
observations_X = [0, disturbance_x(0);
                  1, disturbance_x(1);
                  -1, disturbance_x(-1);
                    ];
              
observations_Y = [0, disturbance_y(0);
                  1, disturbance_y(1);
                  -1, disturbance_y(-1); 
                  ];

start_state.x = [0;0];
start_state.v = [0;0];
start_state.rot = 0;

% get actual disturbance functions
actualdist_x = [];
actualdist_y = [];

for i = 1:numel(x_space)
    actualdist_x = [actualdist_x, disturbance_x(x_space(i))];
end
for i = 1:numel(y_space)
    actualdist_y = [actualdist_y, disturbance_y(y_space(i))];
end
    
for i = 1:20
    fprintf('Running iteration %d...\n', i)
    % get params for each GP
    [fx_bar_star, cov_fx_star] = fit_params(observations_X(:, 1), observations_X(:, 2), x_space, sigma_f, l, assumed_sigma_n);
    [fy_bar_star, cov_fy_star] = fit_params(observations_Y(:, 1), observations_Y(:, 2), y_space, sigma_f, l, assumed_sigma_n);

    % get mean/high/low estimates of function and plot
    % mean is just f_bar_star
    % std dev is the sqrt of the diagonal of cov_f_star
    std_fx_star = sqrt(diag(cov_fx_star));
    std_fy_star = sqrt(diag(cov_fy_star));
    
    

    figure(8)
    clf(8)
    subplot(2, 1, 1)
    hold on
    plot(x_space, actualdist_x, 'k-'); % actual
    xscatter = scatter(observations_X(:, 1), observations_X(:, 2), 'ko'); % observations
    plot(x_space, fx_bar_star, 'b--'); % mean
    upper_x = (fx_bar_star + sigma_confidence * std_fx_star)';
    lower_x = (fx_bar_star - sigma_confidence * std_fx_star)';
    fill_Ax = [x_space, fliplr(x_space)];
    fill_Bx = [lower_x, fliplr(upper_x)];
    fill(fill_Ax, fill_Bx, 'k', 'facealpha', 0.2, 'edgealpha', 0);
    title('X-dependent Disturbance')
    axis([-plot_cutoff, plot_cutoff, -plot_cutoff, plot_cutoff])
    hold off
    subplot(2, 1, 2)
    hold on
    plot(y_space, actualdist_y, 'k-'); % actual
    yscatter = scatter(observations_Y(:, 1), observations_Y(:, 2), 'ko'); % observations
    plot(y_space, fy_bar_star, 'b--'); % mean
    upper_y = (fy_bar_star + sigma_confidence * std_fy_star)';
    lower_y = (fy_bar_star - sigma_confidence * std_fy_star)';
    fill_Ay = [y_space, fliplr(y_space)];
    fill_By = [lower_y, fliplr(upper_y)];
    fill(fill_Ay, fill_By, 'k', 'facealpha', 0.2, 'edgealpha', 0);
    title('Y-dependent Disturbance')
    axis([-plot_cutoff, plot_cutoff, -plot_cutoff, plot_cutoff])
    hold off
    
    
    
    % find largest forward invariant set based on current gps
    tic;
    fprintf('Forward Invariant Set...');
    [x_finv_range, x_next_finv, safety_AX, safety_BX] = find_largest_finv_range(x_space, fx_bar_star, cov_fx_star, [-10, 6]);
    [y_finv_range, y_next_finv, safety_AY, safety_BY] = find_largest_finv_range(y_space, fy_bar_star, cov_fy_star, [-10, 6]);
    fprintf('found.\n');
    toc
    % highlight range in disturbance graphs
    figure(8)
    hold on
    subplot(2, 1, 1)
    rp_x = patch([x_finv_range(1), x_finv_range(2), x_finv_range(2), x_finv_range(1)], [min(ylim) *[1 1], max(ylim)*[1 1]], [0 0 1]);
    rp_x.FaceAlpha = 0.2;
    rp_x.EdgeAlpha = 0;
    legend('Actual', 'Observations', 'Estimated Mean', 'Confidence Bound', 'Forward Invariant Range');
    hold off
    subplot(2, 1, 2)
    hold on
    rp_y = patch([y_finv_range(1), y_finv_range(2), y_finv_range(2), y_finv_range(1)], [min(ylim) *[1 1], max(ylim)*[1 1]], [0 0 1]);
    rp_y.FaceAlpha = 0.2;
    rp_y.EdgeAlpha = 0;
    legend('Actual', 'Observations', 'Estimated Mean', 'Confidence Bound', 'Forward Invariant Range');
    hold off
    
    finv_rect = [x_finv_range(1), y_finv_range(1);
                         x_finv_range(2), y_finv_range(1);
                         x_finv_range(2), y_finv_range(2);
                         x_finv_range(1), y_finv_range(2);
                         x_finv_range(1), y_finv_range(1)];

    figure(7)
    finv_rectplot = plot(finv_rect(:, 1), finv_rect(:, 2), 'b');
    hold on
    quiver(sampling_points_X, sampling_points_Y, sampling_points_U, sampling_points_V);
    
    % shrink hyperrectangle a bit to get points to observe
    center_x = (x_finv_range(2) - x_finv_range(1))/2 + x_finv_range(1);
    center_y = (y_finv_range(2) - y_finv_range(1))/2 + y_finv_range(1);
    initial_hyperrect_relcenter = [finv_rect(:, 1) - center_x, finv_rect(:, 2) - center_y];
    %shrunk_hr_relcenter = sign(initial_hyperrect_relcenter) .* (abs(initial_hyperrect_relcenter) - 0.25); %* 0.85
    shrunk_hr_relcenter = initial_hyperrect_relcenter .* 0.85;
    shrunk_hyperrect = [shrunk_hr_relcenter(:, 1) + center_x, shrunk_hr_relcenter(:, 2) + center_y];

    scatter(shrunk_hyperrect(1:4, 1), shrunk_hyperrect(1:4, 2), '*');
    for c = 1:4
        rectangle('Position', [shrunk_hyperrect(c, :) - obj_buffer, 2 *obj_buffer, 2 *obj_buffer], 'Curvature', [1, 1]);
    end
    axis([-plot_cutoff, plot_cutoff, -plot_cutoff, plot_cutoff])
    
    next_rect = [x_next_finv(1), y_next_finv(1);
                         x_next_finv(2), y_next_finv(1);
                         x_next_finv(2), y_next_finv(2);
                         x_next_finv(1), y_next_finv(2);
                         x_next_finv(1), y_next_finv(1)];
    plot(next_rect(:, 1), next_rect(:, 2), 'b--');
                     
    title('Calculated Forward Invariant Set w/ Planned Observation Points');
    
    % obstacles
    rectangle('Position', [6, 0, 1, 6], 'FaceColor', 'k', 'EdgeColor', 'k');
    rectangle('Position', [-plot_cutoff, 6, 2*plot_cutoff, 1], 'FaceColor', 'k', 'EdgeColor', 'k');
    
    
    
    
    % add points to observations
    to_observe = shrunk_hyperrect(1:end-1, :);
    
     
    
    start_state = animate_quad(7, 8, xscatter, yscatter, start_state); 
    
    observations_X = [xscatter.XData; xscatter.YData]';
    observations_Y = [yscatter.XData; yscatter.YData]';
    
    hold off
    
    

    fprintf('Iteration %d Complete, awaiting button press...', i);
    w = waitforbuttonpress;
    fprintf('registered.\n');
end


% Disturbance Functions
function disturbance = disturbance_x(x)
    global assumed_sigma_n

    noise_observations = [-10, -5; 0, -5; 10, -6];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), x, 30, 5, assumed_sigma_n);
end

function disturbance = disturbance_y(y)
    global assumed_sigma_n
    noise_observations = [-10, 0; 0, 5; 10, 6];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), y, 30, 5, assumed_sigma_n);
end

% Gaussian Process models
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

function [range, next, safety_A, safety_B] = find_largest_finv_range(x_space, f_bar_star, cov_f_star, max_space)
    global K1 K2 K3

    
    Afl = [0, 1, 0;
           0, 0, 1;
           K1, K2, K3];
    Bfl = [0; 1; 0];

    [Te,eigs]=eig(Afl);
    col1 = Te(:, 1);
    col2 = real(Te(:, 2));
    col3 = imag(Te(:, 3));
    Tfl = [col1, col2, col3];
    Tinvfl=inv(Tfl);
    
    freeze = true;
    
    rect = -1;

    
    center_thresh = 0.015;
    center = [0, 0, 0, 0, 0, 0];
    center_rect = [-1, -1, -1, 1, 1, 1];
    cdot = flemb_uncertainty_center(center_rect(1:3)', center_rect(4:6)');
    while sum(abs(cdot)) > center_thresh
        options = optimoptions('quadprog','Display','None');
        [tsc,cs]=ode45(@(t,c) flemb_uncertainty_center(c(1:3),c(4:6)),[0:0.005:5],center_rect,options); %embedding system  
       
        center_rect = cs(end, :);
        cdot = flemb_uncertainty_center(center_rect(1:3)', center_rect(4:6)');
    end
    center = [mean([center_rect(1), center_rect(4)]);
            mean([center_rect(2), center_rect(5)]);
            mean([center_rect(3), center_rect(6)])];
    center = [center; center]';
  
    
   % calculate maximum area available to eigenspace
   max_rect = [max_space(1), 0, 0, max_space(2), 0, 0];
   [vertices,~] = get_hyperrect_from_state(max_rect);
   eig_vertices = (Tinvfl * vertices')';
   
   max_range = [ceil(min(eig_vertices(:, 3))),
       floor(max(eig_vertices(:, 3)))];
   max_search = floor(min(abs(max_range - center(3))));
   % note: the above is calculated this way because the z(3) term is the most
   % directly parallel to the original positional term, THIS WILL CHANGE
   % WITH DIFFERENT K VALUES
   
    
    for f = 1:0.25:max_search
        found_rect = false;
        %refine forward invarant rectangle from backward-time embedding dynamics
        es = center + [-f, -f, -f, f, f, f];
        options = optimoptions('quadprog','Display','None');
        edot = flemb_uncertainty(es(end, 1:3)', es(end, 4:6)');
        finv = (min(edot(1:3)>=0) && min(edot(4:6)<=0));
        %f
        
        %SEARCH ALG 0: FIND SEED, ONCE LARGEST SEED FOUND THEN BACKWARDS
        %EMBED TO REFINE
        if finv
            rect = es;
        else
            if rect ~= -1
                [tse,es]=ode45(@(t,e) -1.*flemb_uncertainty(e(1:3),e(4:6)),[0:0.005:0.2],es(end, :),options); %embedding system  
                for fi = 1:numel(es(:, 1))
                    edot = flemb_uncertainty(es(fi, 1:3)', es(fi, 4:6)');
                    if min(edot(1:3)>=0) && min(edot(4:6)<=0)
                       rect = es(fi, :);
                    else
                       found_rect = true;
                       break;
                    end

                end
            end
        end
        
        if found_rect
            break;
        end
        

     end
    
   if rect ~= -1
        % calculate range of x locking x', x'' to 0
        [vertices,~] = get_hyperrect_from_state(rect);
        orig_vertices = (Tfl * vertices')';
        [safety_A, safety_B] = vert2con(orig_vertices);
        
        % get plotted range by setting x_dot, x_ddot to 0, solve for x
        potential_xs = safety_B./safety_A(:, 1);

        range = [max(potential_xs(potential_xs < 0)), min(potential_xs(potential_xs > 0))]; 
   else
        range = [0, 0];
   end
   
   % do it again to find the next finv set out
   rect = -1;
   for n = f:0.25:f+150
        %refine forward invarant rectangle from backward-time embedding dynamics
        es = center + [-n, -n, -n, n, n, n];
        options = optimoptions('quadprog','Display','None');
        edot = flemb_uncertainty(es(end, 1:3)', es(end, 4:6)');
        finv = (min(edot(1:3)>=0) && min(edot(4:6)<=0));
        %f
        
        %just find the next finv rectangle out there
        if finv
            rect = es;
            break;
        end
        
   end
   
   if rect ~= -1
       
        [vertices,~] = get_hyperrect_from_state(rect);
        orig_vertices = (Tfl * vertices')';
        [next_A, next_B] = vert2con(orig_vertices);
        
        % get plotted range by setting x_dot, x_ddot to 0, solve for x
        potential_xs = next_B./next_A(:, 1);

%         
        next = [max(potential_xs(potential_xs < 0)), min(potential_xs(potential_xs > 0))]; 
   else
        next = [0, 0];
   end
   
    function [wmin_L, wmax_L, wmin_R, wmax_R] = get_disturbances(z, zhat)
        if freeze
            wmin_L = [0;0;0];
            wmax_L = [0;0;0];
            wmin_R = [0;0;0];
            wmax_R = [0;0;0];
            for i = 1:3
                z_new = z;
                zhat_new = zhat;
                %freeze element of z
                zhat_new(i) = z(i);
                %get disturbances for that element
                [wmin_L(i), wmax_L(i)] = disturbance_bounds(z_new, zhat_new);
                %now freeze element of zhat
                zhat_new = zhat;
                z_new(i) = zhat(i);
                %get disturbances for that element
                [wmin_R(i), wmax_R(i)] = disturbance_bounds(z_new, zhat_new);
            end
        else
            [wmin, wmax] = disturbance_bounds(z, zhat);
            wmin_L = wmin;
            wmin_R = wmin;
            wmax_L = wmax;
            wmax_R = wmax;
        end
    end

    function cdot=flemb_uncertainty_center(z, zhat)
        edot_temp = flemb_uncertainty(z, zhat);
        cdot = [mean([edot_temp(1), edot_temp(4)]);
            mean([edot_temp(2), edot_temp(5)]);
            mean([edot_temp(3), edot_temp(6)]);];
        cdot = [cdot; cdot];
    end
    
    function edot=flemb_uncertainty(z,zhat) %dsturbance bounds will be obtained from uncertainty bounds
        
        [wmin_L, wmax_L, wmin_R, wmax_R] = get_disturbances(z, zhat);
        
        edot=[decomp(z,zhat,wmin_L,wmax_L);decomp(zhat,z,wmax_R,wmin_R)]; %embedding dynamics are obtained from the decomposition function
    end

    function d = decomp(z, zhat, w, what)
        Afl_trans = Tinvfl * Afl * Tfl;
        Bfl_trans = Tinvfl * Bfl;

        Afl_trans_pos = max(0, Afl_trans);
        for j = 1:3
            Afl_trans_pos(j, j) = Afl_trans(j, j);
        end
        Afl_trans_neg = Afl_trans - Afl_trans_pos;

        Bfl_trans_pos = max(0, Bfl_trans);
        Bfl_trans_neg = min(0, Bfl_trans);


        d = Afl_trans_pos * z + Afl_trans_neg * zhat + Bfl_trans_pos .* w + Bfl_trans_neg .* what;
       
    end

    function [wmin, wmax] = disturbance_bounds(z, zhat)
        % Transform z-coordinates into x-coordinates
        total_z = [z, zhat];
        [z_points, ~] = get_hyperrect_from_state(total_z);

        x_points = (Tfl * z_points')';

        % get hyperrectangle of extreme x-coordinates
        x = min(x_points);
        xhat = max(x_points);
        
        range_min = min(x(1), xhat(1));
        range_max = max(x(1), xhat(1));
        if (range_min < min(x_space) || range_max > max(x_space))
            error('Attempting to measure disturbance outside of known space, expand range');
        end
        
        % get min/max functions
        global sigma_confidence
        std_f_star = sqrt(diag(cov_f_star));
        min_func = f_bar_star - (sigma_confidence * std_f_star);
        max_func = f_bar_star + (sigma_confidence * std_f_star);
        
        % isolate to range we care about (disturbance is function of
        % position)
        
        min_func = min_func(x_space >= range_min & x_space <= range_max);
        max_func = max_func(x_space >= range_min & x_space <= range_max);
        
        
        wmin = min(min_func);
        wmax = max(max_func);
    end

    function [v, f] = get_hyperrect_from_state(etemp)
        v = [etemp(1), etemp(2), etemp(3);
                    etemp(1), etemp(5), etemp(3);
                    etemp(4), etemp(5), etemp(3);
                    etemp(4), etemp(2), etemp(3);
                    etemp(1), etemp(2), etemp(6);
                    etemp(1), etemp(5), etemp(6);
                    etemp(4), etemp(5), etemp(6);
                    etemp(4), etemp(2), etemp(6)];
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    end

end

% Quadcopter Animation Functions
function [xs,ys]=quadplot(xpos,ypos,rot,scale);
    ts=-.5*pi:.2:3/2*pi+.3;
    xtemp=cos(ts);
    ytemp=sin(ts).*cos(ts);

    xs=[.4*xtemp-1,-1,-1,1,1,.4*xtemp+1];
    ys=[.3*ytemp+.4,.4,0,0,.4,.3*ytemp+.4];
    xs=scale*xs;
    ys=scale*ys;
    rotmat=[cos(rot),-sin(rot);sin(rot),cos(rot)];
    newpos=rotmat*[xs;ys];
    xs=newpos(1,:)+xpos;
    ys=newpos(2,:)+ypos;
end

function final_state = animate_quad(quad_fid, dist_fid, xscatter, yscatter, state0)
    global m g actuator_noise to_observe sigma_n noisy_obs
    

    auxstates=[m*g*1]; %feedback linearizing controller has auxilliary states
    states=[state0];
    inputs=[];
    show_traj=false;
    frameskip=2 ;
    animate_accelerate=4;


    T=20;%Time Horizon
    step=0.01; %time step
    i=0;



    o = 1;
    %for t=0:step:T
    t = 0;
    integral_err = [0;0];
    fprintf('Beginning Flight Simulation.\nObjective %d...', o);
    while (o < 6) && (t < T)
        
        currstate=states(end);
        currauxstate=auxstates(end);
        if check_obj(currstate.x, o)
            o = o + 1;
            fprintf('met.\nObjective %d...', o);
            integral_err = [0;0];
        end
        if o < 5
            objective = to_observe(o, :) .* check_safe(currstate);
        else
            objective = [0, 0];
        end
        integral_err = integral_err + step*(currstate.x - objective');
        %u=gethoverinput(currstate,[1;1],K,sensor_noise);
        u_tilde=getfbinput(t,currstate,currauxstate,objective, integral_err);
        u=[currauxstate;u_tilde(2)];
        nextstate=stateupdate_ode(currstate,u,step,actuator_noise);
        nextauxstate=currauxstate+step*u_tilde(1);
        states=[states, nextstate];
        inputs=[inputs;u];
        auxstates=[auxstates,nextauxstate];
        t = t + step;
    end

    figure(quad_fid)
    hold on
    x=state0.x;
    v=state0.v;
    rot=state0.rot;
    [qplotx,qploty]=quadplot(x(1),x(2),rot,.1);
    plot_hand=plot(qplotx,qploty,'LineWidth',2);hold on;
    if show_traj
        xs=[states.x];
        plh2=plot(xs(1,:),xs(2,:),'-');
    end
    drawnow;
    if o < 6
        fprintf('failed to reach in alotted time.');
    else
        fprintf('met.');
    end
    fprintf('\nAnimating flight...');
    o = 1;
    for i=2:length(states)
        if mod(i,frameskip)==0
            x=states(i).x;
            if check_obj(x, o) 
                if o < 5
                    objective = to_observe(o, :);
                    hold off
                    figure(dist_fid)
                    hold on
                    if noisy_obs
                        noise_x = normrnd(0, sigma_n);
                        noise_y = normrnd(0, sigma_n);
                    else
                        noise_x = 0;
                        noise_y = 0;
                    end
                    set(xscatter, 'XData', [xscatter.XData, x(1)], ...
                    'YData', [xscatter.YData, disturbance_x(x(1)) + noise_x]);
                    set(yscatter, 'XData', [yscatter.XData, x(2)], ...
                    'YData', [yscatter.YData, disturbance_y(x(2)) + noise_y]);
                    drawnow;
                    hold off
                    figure(quad_fid)
                    o = o + 1;
                else
                    break;
                end
            end
            v=states(i).v;
            rot=states(i).rot;
            [qplotx,qploty]=quadplot(x(1),x(2),rot,.1);
            set(plot_hand,'XData',qplotx,'YData',qploty);
            drawnow;
            pause(step*frameskip/animate_accelerate);
        end
    end
    hold off
    final_state = states(end);
    fprintf('done.\n', o);
end

function safe = check_safe(state)
    global finv_buffer safety_AX safety_BX safety_AY safety_BY
    x_vec = [state.x(1); state.v(1); 0];
    y_vec = [state.x(2); state.v(2); 0];
    
    x_buffer = safety_BX - (safety_AX * x_vec);
    y_buffer = safety_BY - (safety_AY * y_vec);
    
    buffer_ratio = [min(x_buffer)/finv_buffer, min(y_buffer)/finv_buffer];
    
    safe = min(buffer_ratio, 1);
end

function goal = check_obj(state, o)
    global obj_buffer to_observe
    if o < 5
        objective = to_observe(o, :)';
    else
        objective = [0;0];
    end
    goal = (norm(state - objective) <= obj_buffer);
end

function u_tilde=getfbinput(t,state,auxstate,objective, integral_err)
    %[y_des,ydot_des,yddot_des,ydddot_des]=reffunc_fb(t);
    y_des=[objective(1);objective(2)];
    ydot_des=[0;0];
    yddot_des=[0;0];
    ydddot_des=[0;0];
    global g_act K1 K2 K3
    if any(y_des)
        KI = -40; % integral term
    else
        KI = 0; % if objective is [0, 0], want to revert to safety action
    end
    y=state.x;
    ydot=state.v;
    yddot=[-auxstate*sin(state.rot);auxstate*cos(state.rot)-g_act];
    v=K1*(y-y_des)+K2*(ydot-ydot_des)+K3*(yddot-yddot_des)+ydddot_des + KI*integral_err;
    u_tilde_unsaturated=inv([-sin(state.rot),-auxstate*cos(state.rot);cos(state.rot),-auxstate*sin(state.rot)])*v;
    u_tilde = 100.*atan(0.01*u_tilde_unsaturated);
    %u_tilde = u_tilde_unsaturated;
end

function [y_des,ydot_des,yddot_des,ydddot_des]=hover_fb(t)
    y_des=[1;1];
    ydot_des=[0;0];
    yddot_des=[0;0];
    ydddot_des=[0;0];
end 

function [y_des,ydot_des,yddot_des,ydddot_des]=explore_fb(t, i)
    global to_observe
    y_des=to_observe(i, :)';
    ydot_des=[0;0];
    yddot_des=[0;0];
    ydddot_des=[0;0];
end 

function [y_des,ydot_des,yddot_des,ydddot_des]=circle_fb(t)
    w=2;
    y_des=[cos(w*t);sin(w*t)];
    ydot_des=[-w*sin(w*t);w*cos(w*t)];
    yddot_des=[-w^2*cos(w*t);-w^2*sin(w*t)];
    ydddot_des=[w^3*sin(w*t);-w^3*cos(w*t)];
end

% Simulation Functions
function statenew=stateupdate_ode(state,u,step,noise)
    global m_act
    global g_act
    x=state.x;
    v=state.v;
    rot=state.rot;
    
    %%Inputs
    tau=u(1)+noise*randn;
    omega=u(2)+noise*randn;
      
    %build y variable for ode function
    y0=[x;v;rot];
    
    %Call ODE45
    [~,y]=ode45(@(t,y) state_derivative(t,y,tau,omega),[0,step],y0);
    
    %Build new state from y variable
    statenew.x=y(end,1:2)';
    statenew.v=y(end,3:4)';
    statenew.rot=y(end,5);
end
function dydt = state_derivative(t,y,tau,omega)
    global m_act
    global g_act
    x=y(1:2);
    v=y(3:4);
    rot=y(5);
    xdot=v;
    vdot=[-1/m_act*tau*sin(rot) + disturbance_x(x(1));1/m_act*tau*cos(rot)-m_act*g_act + disturbance_y(x(2))]; % add in state-dependent disturbance
    rotdot=omega;
    dydt=[xdot;vdot;rotdot];
end