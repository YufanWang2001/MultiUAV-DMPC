
%% Multi-UAV ET-DMPC All-in-One (CasADi-enabled, Paper-Ready)
% Author: Yufan Wang
% Date: 2025-08-08
%
% This single MATLAB file runs multi-UAV cooperative path planning with
% dynamic obstacle avoidance under synchronous DMPC (baseline) and
% event-triggered DMPC (ET-DMPC). It supports a CasADi-based NMPC solver
% (IPOPT) or a fallback fmincon solver.
%
% ======================== IMPORTANT ========================
% 1) Install CasADi for MATLAB and set the correct path below:
%       sim.casadi.enabled = true;
%       sim.casadi.path = 'D:\MATLAB\casadi';   % <-- your path
% 2) Requires Optimization Toolbox for the fmincon fallback.
% ===========================================================

%% === 0) CasADi Setup ===
sim.casadi.enabled = true;                  
sim.casadi.path    = 'D:\MATLAB\casadi';    
if sim.casadi.enabled
    if exist(sim.casadi.path, 'dir')
        addpath(genpath(sim.casadi.path));
        try
            import casadi.*
        catch
            warning('CasADi not found at %s. Falling back to fmincon.', sim.casadi.path);
            sim.casadi.enabled = false;
        end
    else
        warning('CasADi path does not exist: %s. Falling back to fmincon.', sim.casadi.path);
        sim.casadi.enabled = false;
    end
end

%% ============ 1. Global Parameters ============
rng(42);

% Fleet & simulation
sim.N              = 5;       % 1 leader + N-1 followers
sim.Ts             = 0.1;     % s
sim.T_total        = 300;     % steps
sim.Hp             = 12;      % prediction horizon
sim.Hc             = 4;       % control horizon (<= Hp)

% Limits
sim.max_accel      = 4.0;     % |a| <= max_accel
sim.max_vel        = 8.0;     % for plotting/metrics only
sim.min_safe_dist  = 1.2;     % UAV/obstacle margin

% Event-trigger parameters
sim.trigger.mode         = 'combined';    % 'combined' | 'state' | 'input'
sim.trigger.threshold    = 0.45;
sim.trigger.rel          = 0.05;
sim.trigger.dwell        = 3;
sim.trigger.w_state      = [1;1;1; 0.3;0.3;0.3];
sim.trigger.w_input      = 0.2;
sim.trigger.safety_margin= 0.5;

% Costs
sim.W_formation    = 20;
sim.W_avoidance    = 8;                 % (used in fmincon cost; CasADi uses hard constraints for clearance)
sim.R              = 0.05*eye(3);       % control effort

% Formation offsets (columns per UAV; col 1 is leader [0;0;0])
formation_offsets = [ ...
     0,  1.8, -1.8,  0.0,  0.0;
     0,  0.0,  0.0,  1.8, -1.8;
     0,  0.0,  0.0,  0.0,  0.0
];

% Map & plotting
map.xlim = [-15, 35]; map.ylim = [-20, 25]; map.zlim = [0, 15];
map.aspect = [1 1 0.6]; map.fig_dpi = 300; map.linewidth = 1.8; map.fontsize = 12;
outdir = fullfile(pwd, 'fig_out'); if ~exist(outdir,'dir'); mkdir(outdir); end
set(0, 'DefaultAxesFontSize', map.fontsize, 'DefaultAxesLineWidth', 1.0);
set(0, 'DefaultLineLineWidth', map.linewidth);

%% ============ 2. Scenario Definitions ============
leader_ref = @(t, p0, p1, T) [linspace(p0(1), p1(1), T+1); ...
                              linspace(p0(2), p1(2), T+1); ...
                              linspace(p0(3), p1(3), T+1)];
p0 = [-10; -10;  2.5];
p1 = [  30;  15;  4.0];
leader_traj = leader_ref(sim.Ts*(0:sim.T_total), p0, p1, sim.T_total);

initial_states = zeros(6, sim.N);
for i = 1:sim.N, initial_states(1:3,i) = p0 + formation_offsets(:,i); end

% Obstacles
obstacles_static = {
    struct('type','static',  'center', [-4;  -6; 3.5], 'radius', 2.0)
    struct('type','static',  'center', [ 8;   0; 3.0], 'radius', 2.2)
    struct('type','static',  'center', [18;  -8; 3.5], 'radius', 2.0)
    struct('type','static',  'center', [24;   8; 4.5], 'radius', 2.2)
};
dyn1 = @(k) [-8 + 0.10*k;  -12 + 0.08*k; 3.0];
dyn2 = @(k) [ 10 - 0.08*k;   12 - 0.10*k; 4.0];
dyn3 = @(k) [ 18;             -6 + 0.07*k; 3.5];
obstacles_dynamic = {
    struct('type','dynamic','center_func', dyn1, 'radius', 1.8)
    struct('type','dynamic','center_func', dyn2, 'radius', 1.8)
    struct('type','dynamic','center_func', dyn3, 'radius', 1.6)
};
S1.obstacles = obstacles_static;
S2.obstacles = [obstacles_static; obstacles_dynamic];
S3.obstacles = S2.obstacles;

%% ============ 3. Runs ============
[states_S1, inputs_S1, flags_S1] = run_dmpc_scenario(initial_states, leader_traj, formation_offsets, S1.obstacles, sim, 'S1: Static (Sync)', false);
[states_S2, inputs_S2, flags_S2] = run_dmpc_scenario(initial_states, leader_traj, formation_offsets, S2.obstacles, sim, 'S2: Dynamic (Sync)', false);
[states_S3_sync, inputs_S3_sync, flags_S3_sync] = run_dmpc_scenario(initial_states, leader_traj, formation_offsets, S3.obstacles, sim, 'S3a: Dynamic (Sync)', false);
[states_S3_et,   inputs_S3_et,   flags_S3_et  ] = run_dmpc_scenario(initial_states, leader_traj, formation_offsets, S3.obstacles, sim, 'S3b: Dynamic (ET)', true);

%% ============ 4. Metrics & Figures ============
[E11_sync,E12_sync,E2_sync,E3_sync] = calculate_metrics(states_S3_sync, inputs_S3_sync, S3.obstacles, leader_traj, formation_offsets, sim);
[E11_et,  E12_et,  E2_et,  E3_et  ] = calculate_metrics(states_S3_et,   inputs_S3_et,   S3.obstacles, leader_traj, formation_offsets, sim);

fig1 = figure('Name','S1 Trajectories','Position',[100 80 1400 520],'Color','w');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile(1); plot_scene(states_S1, S1.obstacles, leader_traj, sim, map, 'S1: 3D View'); view(38,22);
nexttile(2); plot_topview(states_S1, S1.obstacles, leader_traj, sim, map, 'S1: Plan View');
safe_export(fig1, fullfile(outdir,'S1_static_sync.png'), map.fig_dpi);

fig2 = figure('Name','S2 Trajectories','Position',[100 80 1400 520],'Color','w');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile(1); plot_scene(states_S2, S2.obstacles, leader_traj, sim, map, 'S2: 3D View'); view(38,22);
nexttile(2); plot_topview(states_S2, S2.obstacles, leader_traj, sim, map, 'S2: Plan View');
safe_export(fig2, fullfile(outdir,'S2_dynamic_sync.png'), map.fig_dpi);

fig3 = figure('Name','S3 Trajectories (Sync vs ET)','Position',[90 70 1400 520],'Color','w');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile(1); plot_scene(states_S3_sync, S3.obstacles, leader_traj, sim, map, 'S3a: Sync'); view(38,22);
nexttile(2); plot_scene(states_S3_et,   S3.obstacles, leader_traj, sim, map, 'S3b: ET');   view(38,22);
safe_export(fig3, fullfile(outdir,'S3_dynamic_traj_compare.png'), map.fig_dpi);

fig4 = figure('Name','S3 Metrics','Position',[100 60 1300 800],'Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
k = 1:sim.T_total;
nexttile; hold on; grid on;
plot(k,E11_sync,'DisplayName','Sync'); plot(k,E11_et,'DisplayName','ET'); title('Velocity Consensus E_{1,1}(k)'); xlabel('k'); ylabel('Error'); legend('Location','northeast');
nexttile; hold on; grid on;
plot(k,E12_sync,'DisplayName','Sync'); plot(k,E12_et,'DisplayName','ET'); title('Formation Error E_{1,2}(k)'); xlabel('k'); ylabel('Error'); legend('Location','northeast');
nexttile; hold on; grid on;
plot(k,E2_sync,'DisplayName','Sync');  plot(k,E2_et,'DisplayName','ET');  title('Input Smoothness E_{2}(k)');   xlabel('k'); ylabel('Cost'); legend('Location','northeast');
nexttile; hold on; grid on;
plot(k,E3_sync,'DisplayName','Sync');  plot(k,E3_et,'DisplayName','ET');  title('Safety Margin E_{3}(k)');      xlabel('k'); ylabel('Min dist [m]'); legend('Location','northeast');
safe_export(fig4, fullfile(outdir,'S3_dynamic_metrics.png'), map.fig_dpi);

% Savings (by triggers)
solves_sync = sim.N*sim.T_total;
solves_et   = nnz(flags_S3_et);
fprintf('\n[ET-DMPC Savings]\n  Solves (Sync) = %d | (ET) = %d | Saving = %.1f %%\n\n', solves_sync, solves_et, 100*(1 - solves_et/max(1,solves_sync)));

%% ================== Functions ==================
function [all_states, all_inputs, trigger_flags] = run_dmpc_scenario(initial_states, leader_traj, formation_offsets, obstacles, sim, title_str, is_event_triggered)
    all_states  = zeros(6, sim.N, sim.T_total);
    all_inputs  = zeros(3, sim.N, sim.T_total);
    trigger_flags = zeros(sim.N, sim.T_total);

    current_states = initial_states;
    last_inputs    = zeros(3, sim.N);
    last_states    = current_states;
    last_update_k  = -inf(1, sim.N);
    last_u_seq     = repmat({zeros(3*sim.Hc,1)}, 1, sim.N);

    % Pre-compute obstacle centers over horizon per step (for CasADi parameters)
    % We'll build on the fly in the solver call since k changes.

    for k = 1:sim.T_total
        u_k = zeros(3, sim.N);
        for i = 1:sim.N
            if ~is_event_triggered
                u0 = shift_warmstart(last_u_seq{i}, sim);
                u_full = dmpc_controller(i, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0);
                u_k(:,i) = u_full; last_inputs(:,i) = u_full; trigger_flags(i,k) = 1;
                last_u_seq{i} = pack_warmstart(u_full, u0, sim);
            else
                u0 = shift_warmstart(last_u_seq{i}, sim);
                u_full = dmpc_controller(i, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0);

                dx = current_states(:,i) - last_states(:,i);
                m_state = norm(sim.trigger.w_state .* dx);
                m_input = norm(u_full - last_inputs(:,i));
                rel = sim.trigger.rel * max(1e-6, norm(current_states(:,i)));
                base_th = sim.trigger.threshold + rel;
                near_obst = is_near_obstacle(current_states(1:3,i), obstacles, k, sim, sim.trigger.safety_margin);
                dwell_ok  = (k - last_update_k(i)) >= sim.trigger.dwell;

                switch lower(sim.trigger.mode)
                    case 'state', trig_val = m_state;
                    case 'input', trig_val = m_input;
                    otherwise,    trig_val = m_state + sim.trigger.w_input*m_input;
                end

                if (trig_val > base_th && dwell_ok) || near_obst
                    u_k(:,i) = u_full;
                    last_inputs(:,i) = u_full;
                    trigger_flags(i,k) = 1;
                    last_update_k(i) = k;
                    last_u_seq{i} = pack_warmstart(u_full, u0, sim);
                else
                    u_k(:,i) = last_inputs(:,i);
                end
            end
        end
        % Integrate dynamics
        next_states = zeros(size(current_states));
        for i = 1:sim.N
            next_states(:,i) = update_dynamics(current_states(:,i), u_k(:,i), sim.Ts);
        end
        all_states(:,:,k) = current_states;
        all_inputs(:,:,k) = u_k;
        last_states = current_states; current_states = next_states;
    end
end

function near = is_near_obstacle(pos, obstacles, k, sim, margin)
    near = false;
    for oi = 1:numel(obstacles)
        obs = obstacles{oi};
        if strcmp(obs.type,'dynamic'), c = obs.center_func(k); else, c = obs.center; end
        if norm(pos - c) < (obs.radius + sim.min_safe_dist + margin)
            near = true; return;
        end
    end
end

function u0_shift = shift_warmstart(u0_prev, sim)
    if isempty(u0_prev) || numel(u0_prev) ~= 3*sim.Hc
        u0_shift = zeros(3*sim.Hc,1); return;
    end
    U = reshape(u0_prev,3,sim.Hc);
    U = [U(:,2:end), U(:,end)];
    u0_shift = U(:);
end

function u_next = pack_warmstart(u_now, u0_prev, sim)
    if isempty(u0_prev) || numel(u0_prev) ~= 3*sim.Hc
        U = repmat(u_now,1,sim.Hc);
    else
        U = reshape(u0_prev,3,sim.Hc);
        U(:,1) = u_now;
    end
    u_next = U(:);
end

function u = dmpc_controller(id, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0)
    if sim.casadi.enabled
        u = dmpc_controller_casadi(id, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0);
    else
        u = dmpc_controller_fmincon(id, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0);
    end
end

function u = dmpc_controller_fmincon(id, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0)
    if nargin < 8 || isempty(u0), u0 = zeros(3*sim.Hc,1); end
    obj = @(u_seq) compute_cost(u_seq, id, current_states, leader_traj, formation_offsets, obstacles, k, sim);
    nlc = @(u_seq) nonlinear_constraints(u_seq, id, current_states, obstacles, k, sim);
    lb  = repmat(-sim.max_accel*ones(3,1), sim.Hc, 1);
    ub  = repmat( sim.max_accel*ones(3,1), sim.Hc, 1);
    opts = optimoptions('fmincon','Algorithm','sqp','Display','off', ...
                        'MaxIterations', 60, 'OptimalityTolerance',1e-3, 'StepTolerance',1e-6);
    [u_opt, ~, exitflag] = fmincon(obj, u0, [],[],[],[], lb, ub, nlc, opts);
    if exitflag <= 0 || any(isnan(u_opt)), u = zeros(3,1); else, u = u_opt(1:3); end
end

function u = dmpc_controller_casadi(id, current_states, leader_traj, formation_offsets, obstacles, k, sim, u0)
    % Persistent builder: compile once per (Hp,Hc,obsN)
    persistent cache
    obsN = numel(obstacles);
    key = sprintf('Hp%d_Hc%d_obs%d', sim.Hp, sim.Hc, obsN);

    if isempty(cache) || ~isfield(cache, key)
        import casadi.*
        A = [1 0 0 sim.Ts 0  0;
             0 1 0 0  sim.Ts 0;
             0 0 1 0  0  sim.Ts;
             0 0 0 1  0  0;
             0 0 0 0  1  0;
             0 0 0 0  0  1];
        B = [0.5*sim.Ts^2*eye(3); sim.Ts*eye(3)];
        A = MX(A); B = MX(B);

        opti = casadi.Opti();

        U = opti.variable(3, sim.Hc);               % decision variables
        x0 = opti.parameter(6,1);                    % current state
        REF = opti.parameter(3, sim.Hp);             % reference positions over horizon
        OC  = opti.parameter(3*sim.Hp, obsN);        % obstacle centers stacked over horizon (rows)
        RAD = opti.parameter(obsN,1);                % obstacle radii

        % Rollout
        X = MX(6, sim.Hp);
        X(:,1) = x0;
        for h = 1:sim.Hp-1
            uh = U(:, min(h, sim.Hc));
            X(:,h+1) = A*X(:,h) + B*uh;
        end

        % Cost: formation + control (clearance enforced by hard constraints)
        J_form = 0;
        for h = 1:sim.Hp
            e = X(1:3,h) - REF(:,h);
            J_form = J_form + (e.'*e);
        end
        J_u = 0;
        for h = 1:sim.Hc
            J_u = J_u + U(:,h).'*MX(sim.R)*U(:,h);
        end
        J = sim.W_formation*J_form + J_u;
        opti.minimize(J);

        % Constraints: input bounds
        opti.subject_to( vec(U) <=  sim.max_accel );
        opti.subject_to( vec(U) >= -sim.max_accel );

        % Constraints: obstacle clearance
        for h = 1:sim.Hp
            pos = X(1:3,h);
            for oi = 1:obsN
                oc = OC(3*(h-1)+1:3*h, oi);
                thr2 = (RAD(oi) + sim.min_safe_dist)^2;
                opti.subject_to( sum1((pos - oc).*(pos - oc)) >= thr2 );
            end
        end

        % Solver
        p_opts = struct('print_time', false);
        ipopts = struct('print_level', 0, 'max_iter', 400, 'tol', 1e-4);
        opti.solver('ipopt', p_opts, ipopts);

        cache.(key).opti = opti;
        cache.(key).U = U; cache.(key).x0 = x0; cache.(key).REF = REF; cache.(key).OC = OC; cache.(key).RAD = RAD;
    else
        opti = cache.(key).opti;
        U = cache.(key).U; x0 = cache.(key).x0; REF = cache.(key).REF; OC = cache.(key).OC; RAD = cache.(key).RAD;
    end

    % Prepare parameters
    ref_seq = zeros(3, sim.Hp);
    for h = 1:sim.Hp
        ref_seq(:,h) = leader_traj(:, min(k+h, size(leader_traj,2))) + formation_offsets(:,id);
    end

    obsN = numel(obstacles);
    oc_param = zeros(3*sim.Hp, obsN);
    rad_param = zeros(obsN,1);
    for oi = 1:obsN
        rad_param(oi) = obstacles{oi}.radius;
        for h = 1:sim.Hp
            if strcmp(obstacles{oi}.type,'dynamic')
                oc_param(3*(h-1)+1:3*h, oi) = obstacles{oi}.center_func(k+h);
            else
                oc_param(3*(h-1)+1:3*h, oi) = obstacles{oi}.center;
            end
        end
    end

    % Set parameter values
    opti.set_value(x0, current_states(:,id));
    opti.set_value(REF, ref_seq);
    opti.set_value(OC, oc_param);
    opti.set_value(RAD, rad_param);

    % Warm start
    if nargin < 8 || isempty(u0), u0 = zeros(3*sim.Hc,1); end
    opti.set_initial(vec(U), u0);

    % Solve
    try
        sol = opti.solve();
        Ustar = sol.value(U);
        u = Ustar(:,1);
    catch ME
        warning('CasADi solve failed at k=%d UAV=%d: %s. Using zero input.', k, id, ME.message);
        u = zeros(3,1);
        % Also try to reset solver state for next call
        opti.set_initial(vec(U), zeros(3*sim.Hc,1));
    end
end

function cost = compute_cost(u_seq, id, current_states, leader_traj, formation_offsets, obstacles, k, sim)
    X = zeros(6, sim.Hp); X(:,1) = current_states(:,id);
    for h = 1:sim.Hp-1
        if h <= sim.Hc, u_h = u_seq(3*(h-1)+1:3*h);
        else,           u_h = u_seq(3*(sim.Hc-1)+1:3*sim.Hc); end
        X(:,h+1) = update_dynamics(X(:,h), u_h, sim.Ts);
    end
    J_form = 0;
    for h = 1:sim.Hp
        ref = leader_traj(:, min(k+h, size(leader_traj,2))) + formation_offsets(:,id);
        e   = (X(1:3,h) - ref);
        J_form = J_form + e.'*e;
    end
    % Soft avoidance penalty
    J_avoid = 0;
    for h = 1:sim.Hp
        pos = X(1:3,h);
        for oi = 1:numel(obstacles)
            obs = obstacles{oi};
            if strcmp(obs.type,'dynamic'), oc = obs.center_func(k+h); else, oc = obs.center; end
            d   = norm(pos - oc);
            thr = obs.radius + sim.min_safe_dist;
            if d < thr, J_avoid = J_avoid + (thr - d)^2; end
        end
    end
    J_u = u_seq.' * (kron(eye(sim.Hc), sim.R)) * u_seq;
    cost = sim.W_formation*J_form + sim.W_avoidance*J_avoid + J_u;
end

function [c, ceq] = nonlinear_constraints(u_seq, id, current_states, obstacles, k, sim)
    c = []; ceq = [];
    X = zeros(6, sim.Hp); X(:,1) = current_states(:,id);
    for h = 1:sim.Hp-1
        if h <= sim.Hc, u_h = u_seq(3*(h-1)+1:3*h);
        else,           u_h = u_seq(3*(sim.Hc-1)+1:3*sim.Hc); end
        X(:,h+1) = update_dynamics(X(:,h), u_h, sim.Ts);
    end
    % Clearance constraints
    for h = 1:sim.Hp
        pos = X(1:3,h);
        for oi = 1:numel(obstacles)
            obs = obstacles{oi};
            if strcmp(obs.type,'dynamic'), oc = obs.center_func(k+h); else, oc = obs.center; end
            d2 = sum((pos - oc).^2);
            c = [c; (obs.radius + sim.min_safe_dist)^2 - d2];
        end
    end
end

function x_next = update_dynamics(x, u, Ts)
    A = [1 0 0 Ts 0  0;
         0 1 0 0  Ts 0;
         0 0 1 0  0  Ts;
         0 0 0 1  0  0;
         0 0 0 0  1  0;
         0 0 0 0  0  1];
    B = [0.5*Ts^2*eye(3); Ts*eye(3)];
    x_next = A*x + B*u;
end

function [E11, E12, E2, E3] = calculate_metrics(all_states, all_inputs, obstacles, leader_traj, formation_offsets, sim)
    T = sim.T_total; N = sim.N;
    E11 = zeros(1,T); E12 = zeros(1,T); E2 = zeros(1,T); E3 = zeros(1,T);
    for k = 1:T
        V = squeeze(all_states(4:6,:,k));
        P = squeeze(all_states(1:3,:,k));
        v_err = 0; for i = 1:N, for j = i+1:N, v_err = v_err + norm(V(:,i) - V(:,j))^2; end, end
        E11(k) = v_err;
        form_err = 0; for i = 1:N, ref = leader_traj(:,k) + formation_offsets(:,i); form_err = form_err + norm(P(:,i) - ref)^2; end
        E12(k) = form_err;
        U = squeeze(all_inputs(:,:,k)); if ~isempty(U), E2(k) = sum(vecnorm(U,2,1)); else, E2(k) = 0; end
        min_uav_uav = inf;
        for i = 1:N, for j = i+1:N
            d = norm(P(:,i) - P(:,j)); if d < min_uav_uav, min_uav_uav = d; end
        end, end
        min_uav_obs = inf;
        for i = 1:N
            for oi = 1:numel(obstacles)
                obs = obstacles{oi};
                if strcmp(obs.type,'dynamic'), oc = obs.center_func(k); else, oc = obs.center; end
                d = norm(P(:,i) - oc) - obs.radius; if d < min_uav_obs, min_uav_obs = d; end
            end
        end
        E3(k) = min(min_uav_obs, min_uav_uav);
    end
end

function plot_scene(all_states, obstacles, leader_traj, sim, map, ttl)
    T = sim.T_total; N = sim.N;
    ax = gca; hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    xlim(map.xlim); ylim(map.ylim); zlim(map.zlim); daspect(map.aspect);
    title(ttl,'Interpreter','none');
    draw_obstacles(obstacles);
    plot3(leader_traj(1,1:T), leader_traj(2,1:T), leader_traj(3,1:T), '-', 'Color',[0.6 0.6 0.6], 'LineWidth',1.0, 'DisplayName','Leader Ref');
    cmap = lines(N);
    for i = 1:N
        px = squeeze(all_states(1,i,1:T));
        py = squeeze(all_states(2,i,1:T));
        pz = squeeze(all_states(3,i,1:T));
        plot3(px,py,pz,'-','Color',cmap(i,:),'LineWidth',1.8,'DisplayName',sprintf('UAV %d',i));
        plot3(px(1),py(1),pz(1),'o','Color',cmap(i,:),'MarkerFaceColor',cmap(i,:),'HandleVisibility','off');
        plot3(px(end),py(end),pz(end),'s','Color',cmap(i,:),'MarkerFaceColor',cmap(i,:),'HandleVisibility','off');
    end
    legend('Location','northeastoutside'); set(gcf,'Color','w');
end

function plot_topview(all_states, obstacles, leader_traj, sim, map, ttl)
    T = sim.T_total; N = sim.N;
    ax = gca; hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    xlabel('x [m]'); ylabel('y [m]'); title(ttl,'Interpreter','none');
    xlim(map.xlim); ylim(map.ylim); pbaspect([1 1 1]);
    for oi = 1:numel(obstacles)
        obs = obstacles{oi};
        if isfield(obs,'type') && strcmp(obs.type,'dynamic')
            kk = 1:10:T; cc = arrayfun(@(kkk) obs.center_func(kkk), kk, 'UniformOutput', false);
            cc = cat(2, cc{:});
            plot(cc(1,:), cc(2,:), ':', 'Color', [0.3 0.3 0.3], 'LineWidth',1.0, 'DisplayName','Dyn obst path');
            draw_circle(cc(1,end), cc(2,end), obs.radius, [0.85 0.35 0.35], 0.12);
        else
            draw_circle(obs.center(1), obs.center(2), obs.radius, [0.35 0.45 0.9], 0.15);
        end
    end
    plot(leader_traj(1,1:T), leader_traj(2,1:T), '-', 'Color',[0.6 0.6 0.6], 'LineWidth',1.0, 'DisplayName','Leader Ref');
    cmap = lines(N);
    for i = 1:N
        px = squeeze(all_states(1,i,1:T));
        py = squeeze(all_states(2,i,1:T));
        plot(px,py,'-','Color',cmap(i,:), 'LineWidth',1.8, 'DisplayName',sprintf('UAV %d',i));
        plot(px(end),py(end),'s','Color',cmap(i,:),'MarkerFaceColor',cmap(i,:),'HandleVisibility','off');
    end
    legend('Location','northeastoutside'); set(gcf,'Color','w');
end

function draw_obstacles(obstacles)
    for oi = 1:numel(obstacles)
        obs = obstacles{oi};
        if strcmp(obs.type,'dynamic'), c = obs.center_func(1); clr = [0.85 0.35 0.35]; alp = 0.25;
        else,                          c = obs.center;        clr = [0.35 0.45 0.9]; alp = 0.35; end
        draw_sphere(c(1),c(2),c(3),obs.radius,clr,alp);
    end
end

function draw_sphere(xc,yc,zc,r,faceColor,faceAlpha)
    [xs,ys,zs] = sphere(20);
    surf(r*xs+xc, r*ys+yc, r*zs+zc, 'EdgeColor','none', 'FaceColor',faceColor, 'FaceAlpha',faceAlpha);
end

function draw_circle(xc,yc,r,faceColor,faceAlpha)
    th = linspace(0,2*pi,60);
    x = xc + r*cos(th); y = yc + r*sin(th);
    patch(x,y,faceColor,'FaceAlpha',faceAlpha,'EdgeColor','none');
end

function safe_export(fig, filepath, dpi)
    try
        exportgraphics(fig, filepath, 'Resolution', dpi);
    catch
        try, print(fig, filepath, '-dpng', sprintf('-r%d', dpi)); catch, end
    end
end
