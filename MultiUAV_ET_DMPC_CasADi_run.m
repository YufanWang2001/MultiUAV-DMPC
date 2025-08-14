% MultiUAV_ET_DMPC_AllInOne_CasADi_PRO.m
% 单文件：多无人机 3D 事件触发 DMPC（CasADi/Ipopt + fmincon 兜底）
% - 起终点：立方体对角
% - 编队：分散排布
% - 障碍：静态+动态（软约束）
% - 指标：一致性/编队/平滑/安全 + 求解/通信统计
% - 导出：分图 PNG、CSV、LaTeX 表格
% 作者：YufanWang   2025/08/08

function MultiUAV_ET_DMPC_AllInOne_CasADi_PRO
clc; close all; clearvars;

%% ================= 用户配置 =================
cfg.casadi_path = 'D:\MATLAB\casadi'; % ← 根据你的安装修改
outdir = fullfile(pwd, 'fig_out'); if ~exist(outdir,'dir'); mkdir(outdir); end

% 随机种子
rng(42);

%% ================= 仿真参数 =================
sim.dt   = 0.2;         % s
sim.T    = 200;         % 迭代步数
sim.Hp   = 12;          % 预测域
sim.Hc   = 3;           % 控制域
sim.state_dim = 6;      % [x y z vx vy vz]
sim.input_dim = 3;      % [ax ay az]

sim.vmax = 4.0;         % m/s 速度上限
sim.umin = -2.0*ones(3,1);  % m/s^2
sim.umax =  2.0*ones(3,1);

% 软约束 & 求解器
sim.safety_margin = 0.6;   % 安全裕度
sim.obs_stride    = 2;     % 避障约束步长（加速）
sim.casadi.W_slack = 50.0; % slack 权重
sim.casadi.cache_version = 1; % 调整以强制重建模型
sim.casadi.ipopts = struct( ...
    'print_time', false, ...
    'ipopt', struct( ...
        'print_level', 0, ...
        'max_iter', 200, ...
        'max_cpu_time', 0.2, ... % 单次求解 CPU 上限（秒）
        'tol', 1e-3, ...
        'constr_viol_tol', 1e-3, ...
        'acceptable_tol', 5e-3, ...
        'acceptable_obj_change_tol', 5e-3, ...
        'warm_start_init_point', 'yes' ...
    ) ...
);

% 事件触发参数（通信）
sim.trigger.threshold      = 0.6;   % 误差范数阈值（相对尺度）
sim.trigger.dwell          = 3;     % 最小驻留步
sim.trigger.near_obst_gain = 0.5;   % 近障降低阈值系数
sim.trigger.stall_speed    = 0.1;   % 低速保护（m/s）

% 代价权重（论文中可给出）
sim.W_track = blkdiag(2.0*eye(3), 0.2*eye(3));  % 位置>速度
sim.W_u     = 0.05*eye(3);
sim.W_du    = 0.05*eye(3);

% 领航速度平滑，跟随偏置
sim.leader_speed = 2.5;   % 目标点速度（m/s）

%% =============== 路径 & 环境 ===============
addpath_if_exist(cfg.casadi_path);

% 工作空间（立方体）
ws.min = [0 0 0]';
ws.max = [30 18 12]';

% 起点在 (min) 邻域，终点在 (max) 邻域（对角）
N = 5; % 1 领航 + 4 跟随
[starts, goals, offsets] = make_diagonal_layout(ws, N);

% 障碍（静态若干 + 1 个缓慢动态）
[obst, obst_dyn] = make_obstacles();

%% =============== 参考轨迹（领航） ===============
leader_traj = make_leader_reference(starts(:,1), goals(:,1), sim);

%% =============== 初始化 ===============
states = zeros(sim.state_dim, N, sim.T+1);
inputs = zeros(sim.input_dim, N, sim.T);

states(:, :, 1) = init_states(starts);

% 事件触发状态
event.last_tx_k = -1e6*ones(1,N);
event.savings_solves = 0;   % 避免求解的次数
comms = zeros(1,N);         % 通信次数

% 指标累计
metrics.Evel = zeros(1, sim.T);
metrics.Eform= zeros(1, sim.T);
metrics.Eu   = zeros(1, sim.T);
metrics.Esafe= zeros(1, sim.T);

%% =============== 双场景对比 ===============
S_sync = run_dmpc_scenario(states(:,:,1), leader_traj, offsets, obst, obst_dyn, sim, false);
S_et   = run_dmpc_scenario(states(:,:,1), leader_traj, offsets, obst, obst_dyn, sim, true );

%% =============== 作图/导出 ===============
export_figures_and_tables(S_sync, S_et, ws, outdir);

fprintf('[Savings] Solves: %.1f%% | Comms: %.1f%%\n', 100*S_et.savings_solves, 100*S_et.savings_comms);
end

%% ================== 核心：运行单场景 ==================
function S = run_dmpc_scenario(states0, leader_traj, offsets, obst, obst_dyn, sim, use_event)
import casadi.*

N  = size(states0,2);
T  = sim.T;

% 结果缓存
states = zeros(sim.state_dim, N, T+1);
inputs = zeros(sim.input_dim, N, T);
flags  = zeros(1,T); % 1: 使用了求解器，0: 复用了上一次输入

states(:,:,1) = states0;

% 通信与事件
last_tx_k = -1e6*ones(1,N);
comms = zeros(1,N);

% 初值 u0
u_last = zeros(sim.input_dim, N);

% 障碍缓冲（加速）：提前计算每个 k 的 OC/RAD 序列
[OC_all, RAD_all] = precompute_obstacles_over_horizon(obst, obst_dyn, sim);

for k=1:T
    % 近障检测用于事件阈值调节
    near_obst = false(1,N);
    for i=1:N
        near_obst(i) = is_near_obstacle(states(1:3,i,k), obst, obst_dyn, sim);
    end

    for i=1:N
        % 构造参考（含编队偏置）
        ref_now = make_reference(leader_traj, offsets(:,i), k, sim);

        % 事件触发：决定是否重算优化或复用 u 上次
        do_solve = true;  % 默认要求解
        if use_event
            err = states(:,i,k) - ref_now(:,1);
            eps = sim.trigger.threshold * (1 + 0.0*norm(ref_now(1:3,1))); %#ok<*NASGU>
            if near_obst(i); eps = eps * sim.trigger.near_obst_gain; end
            if k - last_tx_k(i) < sim.trigger.dwell
                do_solve = false; % 驻留期内不给发
            elseif norm(err(1:3)) < eps && norm(states(4:6,i,k)) > sim.trigger.stall_speed
                do_solve = false; % 误差小且未停滞
            end
        end

        if do_solve
            try
                u = dmpc_controller_casadi(i, states(:,:,k), leader_traj, offsets, OC_all{k}, RAD_all, k, sim, u_last(:,i), []);
            catch ME
                warning('CasADi solve failed at k=%d UAV=%d: %s. Falling back to fmincon.', k, i, ME.message);
                u = dmpc_controller_fmincon(i, states(:,i,k), leader_traj, offsets(:,i), obst, obst_dyn, k, sim, u_last(:,i));
            end
            flags(k) = 1;
            last_tx_k(i) = k;
            comms(i) = comms(i) + 1;
        else
            u = u_last(:,i); % 复用
        end

        % 应用一步输入更新状态
        states(:,i,k+1) = step_dynamics(states(:,i,k), u, sim.dt);
        inputs(:,i,k) = u;
        u_last(:,i) = u;
    end
end

% 指标
[Evel, Eform, Eu, Esafe] = compute_metrics(states, inputs, offsets, obst, obst_dyn, sim);

% 汇总
S.states = states; S.inputs = inputs; S.flags = flags; S.use_event = use_event;
S.metrics = struct('Evel', Evel, 'Eform', Eform, 'Eu', Eu, 'Esafe', Esafe);
S.savings_solves = 1 - nnz(flags)/numel(flags);
S.savings_comms  = 1 - sum(comms)/ (sim.T*size(states,2));
S.comms = comms;
end

%% ================== CasADi DMPC 控制器 ==================
function u = dmpc_controller_casadi(id, current_states, leader_traj, formation_offsets, OC, RAD, k, sim, u0, cache)
import casadi.*

persistent MODEL_CACHE
if isempty(MODEL_CACHE); MODEL_CACHE = struct(); end

key = sprintf('uav%d_dim%dHp%dHc%d', id, sim.state_dim, sim.Hp, sim.Hc);
key = [key '_' num2str(sim.casadi.cache_version,'%.0f')];

if ~isfield(MODEL_CACHE, key)
    opti = casadi.Opti();

    % Decision vars
    X = opti.variable(sim.state_dim, sim.Hp+1);   % states over horizon
    U = opti.variable(sim.input_dim, sim.Hc);     % control moves

    % Parameters
    x0  = opti.parameter(sim.state_dim,1,'x0');            % initial state
    REF = opti.parameter(sim.state_dim, sim.Hp,'REF');     % desired state (pos/vel) per step
    obsN = size(RAD,1);
    if obsN==0; obsN = 1; end % keep shape
    % Flatten obstacle centers to 2D: rows = 3*obsN, cols = Hp
    OCp = opti.parameter(3*obsN, sim.Hp, 'OC');            % flattened obstacle centers (per step)
    RADp= opti.parameter(obsN,1,'RAD');                    % obstacle radii

    % Shared slack per step
    S   = opti.variable(1, sim.Hp);                        % >=0
    opti.subject_to(S(:) >= 0);

    % Dynamics constraints (discrete double-integrator, Euler)
    dt = sim.dt;
    A = [eye(3), dt*eye(3); zeros(3), eye(3)];
    B = [0.5*dt^2*eye(3); dt*eye(3)];

    opti.subject_to(X(:,1) == x0);
    for h=1:sim.Hc
        opti.subject_to(X(:,h+1) == A*X(:,h) + B*U(:,h));
    end
    for h=sim.Hc+1:sim.Hp
        opti.subject_to(X(:,h+1) == A*X(:,h) + B*U(:,sim.Hc)); % hold last input
    end

    % Bounds
    vmax = sim.vmax;
    opti.subject_to(-vmax <= X(4:6, :) <= vmax);
    opti.subject_to(sim.umin <= U <= sim.umax);

    % Obstacle constraints (stride)
    for h=1:sim.Hp
        if mod(h-1, sim.obs_stride)==0
            pos = X(1:3, h+1);
            for oi=1:obsN
                oc  = OCp(3*(oi-1)+(1:3), h);
                rad = RADp(oi);
                thr2 = (rad + sim.safety_margin)^2;
                opti.subject_to( sum1((pos-oc).*(pos-oc)) + S(1,h) >= thr2 );
            end
        end
    end

    % Objective: tracking + formation (pos only) via REF, control effort, smoothness, slack
    J_track = 0;
    for h=1:sim.Hp
        e = X(:,h) - REF(:,h);
        J_track = J_track + e.'*sim.W_track*e;
    end

    J_u = 0;
    for h=1:sim.Hc
        J_u = J_u + U(:,h).'*sim.W_u*U(:,h);
    end
    % smoothness (delta-U)
    for h=2:sim.Hc
        dU = U(:,h) - U(:,h-1);
        J_u = J_u + dU.'*sim.W_du*dU;
    end

    J = J_track + J_u + sim.casadi.W_slack*sum1(S(:));

    opti.minimize(J);

    % solver
    ipopts = sim.casadi.ipopts;
    opti.solver('ipopt', ipopts);

    % cache
    MODEL_CACHE.(key).opti = opti;
    MODEL_CACHE.(key).X = X; MODEL_CACHE.(key).U = U; MODEL_CACHE.(key).S = S;
    MODEL_CACHE.(key).x0 = x0; MODEL_CACHE.(key).REF = REF;
    MODEL_CACHE.(key).OC = OCp; MODEL_CACHE.(key).RAD = RADp;
end

opti = MODEL_CACHE.(key).opti;
X = MODEL_CACHE.(key).X; U = MODEL_CACHE.(key).U; S = MODEL_CACHE.(key).S;
x0 = MODEL_CACHE.(key).x0; REF = MODEL_CACHE.(key).REF;
OCp= MODEL_CACHE.(key).OC; RADp=MODEL_CACHE.(key).RAD;

% Set parameters
opti.set_value(x0, current_states(:,id));
ref = make_reference(leader_traj, formation_offsets(:,id), k, sim);
opti.set_value(REF, ref);

obsN = size(RAD,1);
if obsN==0; obsN=1; end
opti.set_value(RADp, RAD(:));

% Flatten OC into (3*obsN) x Hp
OC_flat = zeros(3*obsN, sim.Hp);
for h=1:sim.Hp
    for oi=1:obsN
        OC_flat(3*(oi-1)+(1:3), h) = OC(:,h, min(oi,size(OC,3)));
    end
end
opti.set_value(OCp, OC_flat);

% Initial guess
opti.set_initial(U(:), repmat(u0, sim.Hc, 1));
opti.set_initial(S, zeros(size(S)));
xinit = current_states(:,id);
opti.set_initial(X(:,1), xinit);
for h=1:sim.Hp
    if h<=sim.Hc
        uinit = u0;
    else
        uinit = zeros(sim.input_dim,1);
    end
    xinit = [eye(3), sim.dt*eye(3); zeros(3), eye(3)]*xinit + [0.5*sim.dt^2*eye(3); sim.dt*eye(3)]*uinit;
    opti.set_initial(X(:,h+1), xinit);
end

% Solve
try
    sol = opti.solve();
    u = sol.value(U(:,1));
catch ME
    if contains(ME.message, 'Maximum_CpuTime_Exceeded') || contains(ME.message,'timed out')
        try
            ipopts = sim.casadi.ipopts;
            ipopts.acceptable_tol = 3e-3;
            ipopts.acceptable_obj_change_tol = 3e-3;
            opti.solver('ipopt', ipopts);
            sol = opti.solve();
            u = sol.value(U(:,1));
            return;
        catch
            rethrow(ME);
        end
    else
        rethrow(ME);
    end
end
end

%% ================== fmincon 兜底 ==================
function u = dmpc_controller_fmincon(id, x, leader_traj, offset_i, obst, obst_dyn, k, sim, u0)
% 简化 1 步 MPC：只优化第一步 U，加入软代价避障
opts = optimoptions('fmincon','Display','off','Algorithm','sqp','MaxIterations',50,'SpecifyObjectiveGradient',false);

lb = sim.umin; ub = sim.umax;

cost = @(u) local_cost(u);
nonl = [];

try
    u = fmincon(cost, u0, [],[],[],[], lb, ub, nonl, opts);
catch
    u = min(max(u0, lb), ub); % 再不行就夹断
end

    function J = local_cost(u)
        % 预测 1 步
        A = [eye(3), sim.dt*eye(3); zeros(3), eye(3)];
        B = [0.5*sim.dt^2*eye(3); sim.dt*eye(3)];
        x1 = A*x + B*u;
        ref = make_reference(leader_traj, offset_i, k, sim);
        e   = x1 - ref(:,1);
        J_track = e.'*sim.W_track*e;

        % 软代价避障（只用 k 的障碍 snapshot）
        [OC1, RAD1] = obstacles_at_k(obst, obst_dyn, k, sim);
        J_obst = 0;
        for oi=1:size(OC1,3)
            d = norm(x1(1:3) - OC1(:,1,oi)) - (RAD1(oi) + sim.safety_margin);
            if d < 0
                J_obst = J_obst + sim.casadi.W_slack * ( -d );
            end
        end
        J_u = u.'*sim.W_u*u;
        J = J_track + J_obst + J_u;
    end
end

%% ================== 指标 ==================
function [Evel, Eform, Eu, Esafe] = compute_metrics(states, inputs, offsets, obst, obst_dyn, sim)
T = size(inputs,3); N = size(inputs,2);
Evel = zeros(1,T); Eform = zeros(1,T); Eu = zeros(1,T); Esafe = zeros(1,T);

for k=1:T
    v = squeeze(states(4:6,:,k));
    vbar = mean(v,2);
    Evel(k) = mean(vecnorm(v - vbar, 2, 1));

    p = squeeze(states(1:3,:,k));
    p1= p(:,1);
    form_err = 0;
    for i=2:N
        form_err = form_err + norm( (p(:,i)-p1) - (offsets(:,i)-offsets(:,1)) );
    end
    Eform(k) = form_err/(N-1);

    u = squeeze(inputs(:,:,k));
    Eu(k) = mean(vecnorm(u,2,1));

    % 安全裕度（负数表示穿越）
    [OCk, RADk] = obstacles_at_k(obst, obst_dyn, k, sim);
    safe_min = inf;
    for i=1:N
        for oi=1:size(OCk,3)
            d = norm(states(1:3,i,k) - OCk(:,1,oi)) - (RADk(oi) + sim.safety_margin);
            safe_min = min(safe_min, d);
        end
    end
    Esafe(k) = safe_min; % 越大越安全
end
end

%% ================== 参考相关 ==================
function ref = make_reference(leader_traj, off_i, k, sim)
% 生成 Hp 步参考（位置+速度）
Hp = sim.Hp; ref = zeros(sim.state_dim, Hp);
for h=1:Hp
    idx = min(k+h, size(leader_traj,2));
    posL = leader_traj(1:3, idx) + off_i;   % 跟随者偏置到领航轨
    velL = leader_traj(4:6, idx);
    ref(:,h) = [posL; velL];
end
end

function leader_traj = make_leader_reference(p0, p1, sim)
% 平滑对角线轨迹（S 曲线）
L = norm(p1 - p0);
Tsim = sim.T + sim.Hp + 10;
leader_traj = zeros(6, Tsim);

% 标称速度（沿线）
v = sim.leader_speed;
for k=1:Tsim
    s = min(1, k/(Tsim-1));
    s = s^2*(3-2*s); % smoothstep
    pos = (1-s)*p0 + s*p1;
    % 数值微分速度（前差分）
    if k>1
        vel = (pos - leader_traj(1:3,k-1))/sim.dt;
        if norm(vel)>v
            vel = vel * (v/norm(vel));
        end
    else
        vel = zeros(3,1);
    end
    leader_traj(:,k) = [pos; vel];
end
end

%% ================== 障碍相关 ==================
function [obst, obst_dyn] = make_obstacles()
% 静态球体：中心、半径、颜色（RGB）
obst = struct('c', {}, 'r', {}, 'col', {});
obst(1).c = [10;  7;  4]; obst(1).r = 1.6; obst(1).col=[0.90 0.35 0.10];
obst(2).c = [15; 12;  8]; obst(2).r = 1.8; obst(2).col=[0.10 0.60 0.90];
obst(3).c = [22;  5; 10]; obst(3).r = 1.5; obst(3).col=[0.20 0.75 0.30];
obst(4).c = [ 8; 14;  6]; obst(4).r = 1.4; obst(4).col=[0.70 0.20 0.80];

% 动态球（缓慢漂移）
obst_dyn = struct('c0', [18;9;5], 'r', 1.6, 'v', [0.01; -0.02; 0.015], 'col', [0.95 0.80 0.15]);
end

function [OC_all, RAD_all] = precompute_obstacles_over_horizon(obst, obst_dyn, sim)
% 每个 k: OC_all{k} = 3 x Hp x M, RAD_all = M x 1
M = numel(obst) + (~isempty(obst_dyn));
RAD_all = zeros(M,1);
if M==0; OC_all = cell(sim.T,1); for k=1:sim.T, OC_all{k}=zeros(3,sim.Hp,1); end; return; end

for k=1:sim.T
    OC = zeros(3, sim.Hp, M);
    for h=1:sim.Hp
        t = (k+h-1)*sim.dt;
        m = 0;
        for i=1:numel(obst)
            m=m+1; OC(:,h,m) = obst(i).c; RAD_all(m)=obst(i).r; %#ok<AGROW>
        end
        if ~isempty(obst_dyn)
            m=m+1; OC(:,h,m) = obst_dyn.c0 + obst_dyn.v*t; RAD_all(m)=obst_dyn.r; %#ok<AGROW>
        end
    end
    OC_all{k} = OC;
end
end

function tf = is_near_obstacle(p, obst, obst_dyn, sim)
[OC, RAD] = obstacles_at_k(obst, obst_dyn, 0, sim); %#ok<ASGLU>
for i=1:size(OC,3)
    if norm(p-OC(:,1,i)) < RAD(i)+ 2.0
        tf = true; return; 
    end
end
tf=false;
end

function [OC, RAD] = obstacles_at_k(obst, obst_dyn, k, sim)
M = numel(obst) + (~isempty(obst_dyn));
if M==0; OC=zeros(3,1,1); RAD=0; return; end
OC = zeros(3,1,M); RAD = zeros(M,1);
m=0;
for i=1:numel(obst)
    m=m+1; OC(:,1,m) = obst(i).c; RAD(m)=obst(i).r;
end
if ~isempty(obst_dyn)
    m=m+1; t=k*sim.dt; OC(:,1,m) = obst_dyn.c0 + obst_dyn.v*t; RAD(m)=obst_dyn.r;
end
end

%% ================== 初始布局 ==================
function [starts, goals, offsets] = make_diagonal_layout(ws, N)
% 分散：绕领航在 x/y/z 上错层
center = (ws.min + ws.max)/2;
span   = ws.max - ws.min;

% 领航起终
p0 = ws.min + 0.15*span; p0(3)=max(p0(3), 1.0);
p1 = ws.max - 0.15*span; p1(3)=min(p1(3), ws.max(3)-0.5);

starts = zeros(3,N); goals=zeros(3,N); offsets=zeros(3,N);
starts(:,1)=p0; goals(:,1)=p1; offsets(:,1)=[0;0;0];

% 四个跟随错层 + 分散
spread = diag([2.5, 2.0, 1.5]);
base   = [ 1  1  0; -1  1  0; 1 -1  0; -1 -1  0]';
for i=2:N
    offsets(:,i) = spread * base(:,i-1);
    starts(:,i)  = p0 + offsets(:,i) + [0.5; -0.3; 0.4].*(i-1);
    goals(:,i)   = p1 + offsets(:,i);
end
end

function X0 = init_states(starts)
N=size(starts,2); X0=zeros(6,N);
for i=1:N
    X0(:,i) = [starts(:,i); 0;0;0];
end
end

function xnext = step_dynamics(x, u, dt)
A = [eye(3), dt*eye(3); zeros(3), eye(3)];
B = [0.5*dt^2*eye(3); dt*eye(3)];
xnext = A*x + B*u;
end

%% ================== 作图与导出 ==================
function export_figures_and_tables(S1, S2, ws, outdir)
% 单独图片（3D、俯视、侧视）+ 指标/资源合成图 + CSV/LaTeX

% 3D 路径
f3d1 = figure('Color','w','Name','Sync-3D'); plot_scene_3d(S1, ws); title('Synchronous DMPC','Interpreter','latex');
saveas(f3d1, fullfile(outdir,'sync_3d.png'));

f3d2 = figure('Color','w','Name','ET-3D'); plot_scene_3d(S2, ws); title('Event-Triggered DMPC','Interpreter','latex');
saveas(f3d2, fullfile(outdir,'et_3d.png'));

% 俯视
ft1 = figure('Color','w','Name','Sync-Top'); plot_scene_view(S1, ws, [0 90]); title('Synchronous DMPC (Top View)','Interpreter','latex');
saveas(ft1, fullfile(outdir,'sync_top.png'));

ft2 = figure('Color','w','Name','ET-Top'); plot_scene_view(S2, ws, [0 90]); title('Event-Triggered DMPC (Top View)','Interpreter','latex');
saveas(ft2, fullfile(outdir,'et_top.png'));

% 侧视（前视）
fs1 = figure('Color','w','Name','Sync-Side'); plot_scene_view(S1, ws, [90 0]); title('Synchronous DMPC (Side View)','Interpreter','latex');
saveas(fs1, fullfile(outdir,'sync_side.png'));

fs2 = figure('Color','w','Name','ET-Side'); plot_scene_view(S2, ws, [90 0]); title('Event-Triggered DMPC (Side View)','Interpreter','latex');
saveas(fs2, fullfile(outdir,'et_side.png'));

% 指标 + 资源
fm = figure('Color','w','Name','Metrics','Position',[100 100 1200 700]);
T = size(S1.inputs,3);
subplot(2,3,1); plot_metric(S1.metrics.Evel,S2.metrics.Evel,'Velocity Consensus $E_{1,1}(k)$');
subplot(2,3,2); plot_metric(S1.metrics.Eform,S2.metrics.Eform,'Formation Error $E_{1,2}(k)$');
subplot(2,3,3); plot_metric(S1.metrics.Eu,  S2.metrics.Eu,  'Input Smoothness $E_{2}(k)$');
subplot(2,3,4); plot_metric(S1.metrics.Esafe,S2.metrics.Esafe,'Safety Margin $E_{3}(k)$');
subplot(2,3,5); bar([sum(S1.flags), sum(S2.flags)]); xticklabels({'Sync','ET'}); ylabel('Number of Solves'); title('Solver Calls'); grid on;
subplot(2,3,6); bar(100*[1-S1.savings_comms, 1-S2.savings_comms]); xticklabels({'Sync','ET'}); ylabel('Comms per Step (%)'); title('Communication Load'); grid on;
saveas(fm, fullfile(outdir,'metrics_and_resources.png'));

% 表格导出
results = table();
results.Scenario = {'Sync'; 'ET'};
results.Mean_Vel_Consensus = [mean(S1.metrics.Evel), mean(S2.metrics.Evel)]';
results.Mean_Formation_Err = [mean(S1.metrics.Eform), mean(S2.metrics.Eform)]';
results.Mean_Input_Norm    = [mean(S1.metrics.Eu),   mean(S2.metrics.Eu)]';
results.Min_Safety_Margin  = [min(S1.metrics.Esafe), min(S2.metrics.Esafe)]';
results.Solves_Reduction   = [S1.savings_solves, S2.savings_solves]';
results.Comms_Reduction    = [S1.savings_comms, S2.savings_comms]';

writetable(results, fullfile(outdir,'baseline_results.csv'));

% LaTeX 简表
g = @(x) sprintf('%.3f',x);
L = ["\\begin{tabular}{lrrrrrr}", ...
     "\\toprule", ...
     "Scenario & $\\overline{E}_{1,1}$ & $\\overline{E}_{1,2}$ & $\\overline{E}_{2}$ & $\\min E_3$ & Saves(Solves) & Saves(Comms) \\ ", ...
     "\\midrule", ...
     sprintf('Sync & %s & %s & %s & %s & %s & %s \\', g(results.Mean_Vel_Consensus(1)), g(results.Mean_Formation_Err(1)), g(results.Mean_Input_Norm(1)), g(results.Min_Safety_Margin(1)), g(results.Solves_Reduction(1)), g(results.Comms_Reduction(1))), ...
     sprintf('ET & %s & %s & %s & %s & %s & %s \\', g(results.Mean_Vel_Consensus(2)), g(results.Mean_Formation_Err(2)), g(results.Mean_Input_Norm(2)), g(results.Min_Safety_Margin(2)), g(results.Solves_Reduction(2)), g(results.Comms_Reduction(2))), ...
     "\\bottomrule", ...
     "\\end{tabular}"];
texfile = fullfile(outdir,'baseline_results.tex');
try
    fid = fopen(texfile,'w'); fprintf(fid,'%s\n', L); fclose(fid);
catch
end
fprintf('LaTeX table written: %s\n', texfile);
end

function plot_metric(a, b, ttl)
plot(a,'LineWidth',1.2); hold on; plot(b,'LineWidth',1.2);
legend({'Sync','ET'},'Location','best'); grid on;
ylabel('Value'); title(ttl,'Interpreter','latex');
end

function plot_scene_3d(S, ws)
T = size(S.inputs,3); N=size(S.inputs,2);
col = lines(N);
hold on; box on; grid on; view(45,25);
for i=1:N
    P = squeeze(S.states(1:3,i,1:T+1));
    plot3(P(1,:),P(2,:),P(3,:),'LineWidth',1.8,'Color',col(i,:));
    plot3(P(1,1),P(2,1),P(3,1),'o','MarkerFaceColor',col(i,:),'MarkerEdgeColor','k');
    plot3(P(1,end),P(2,end),P(3,end),'s','MarkerFaceColor',col(i,:),'MarkerEdgeColor','k');
end
xlim([ws.min(1) ws.max(1)]); ylim([ws.min(2) ws.max(2)]); zlim([ws.min(3) ws.max(3)]);
xlabel('x'); ylabel('y'); zlabel('z');
end

function plot_scene_view(S, ws, ang)
plot_scene_3d(S, ws); view(ang(1), ang(2));
end

function addpath_if_exist(p)
if exist(p,'dir'); addpath(p); end
end
