%% run_dmpc_sci_final.m
% A Comprehensive Simulation System for Multi-UAV DMPC
% Author: Gemini AI
% Date: August 2025
%
% Description:
% This script runs four distinct scenarios for multi-UAV formation control and
% obstacle avoidance, replicating the scenarios and figures from the provided image.
% It is designed to produce high-quality, publication-ready figures for an SCI paper.

clear; clc; close all;

%% ======================= 1. Global Simulation Parameters =======================
sim_params.N = 5;                  % Number of UAVs (1 Leader, 4 Followers)
sim_params.Ts = 0.1;               % Sampling time (s)
sim_params.T_total = 100;          % Total simulation steps
sim_params.max_accel = 5;          % Maximum acceleration (m/s^2)
sim_params.max_vel = 10;           % Maximum velocity (m/s)

% DMPC Controller Parameters
sim_params.Hp = 15;                % Prediction horizon
sim_params.Hc = 5;                 % Control horizon
sim_params.event_trigger_threshold = 0.5; % For ET-DMPC

% DMPC Cost Function Weights (tuned for performance)
sim_params.Q = diag([10, 10, 10, 1, 1, 1]);  % State tracking weight
sim_params.R = diag([0.1, 0.1, 0.1]);         % Control input penalty
sim_params.W_formation = 10;                  % Formation cost weight
sim_params.W_avoidance = 5000;                % Obstacle avoidance cost weight

%% ======================= 2. UAV Initial Conditions and Formation =======================
initial_states = zeros(6, sim_params.N);
initial_states(1:3, 1) = [0; 0; 5]; % Leader's initial position
initial_states(4:6, :) = zeros(3, sim_params.N);

formation_offsets = [
    -2, 0, 0;    % Follower 1 relative to Leader
    2, 0, 0;     % Follower 2
    0, -2, 0;    % Follower 3
    0, 2, 0      % Follower 4
]';

initial_states(1:3, 2:end) = initial_states(1:3, 1) + formation_offsets;

% Leader's Target Trajectory (a smooth helix)
leader_trajectory = @(k) [20 * sin(k * sim_params.Ts * 0.1); ...
                          20 * cos(k * sim_params.Ts * 0.1); ...
                          5 + 3 * sin(k * sim_params.Ts * 0.05)];

%% ======================= 3. Define Obstacle Scenarios from Image =======================
% Obstacle coordinates are taken from the provided image
obstacle_scenarios = cell(1, 4);
obstacle_scenarios{1} = {
    struct('type', 'static', 'center', [25; 11; 3], 'radius', 3), ...
    struct('type', 'static', 'center', [55; -6; 3], 'radius', 3)
};
obstacle_scenarios{2} = {
    struct('type', 'static', 'center', [20; -7; 3], 'radius', 3), ...
    struct('type', 'static', 'center', [50; 11; 3], 'radius', 3)
};
obstacle_scenarios{3} = {
    struct('type', 'static', 'center', [25; -13; 3], 'radius', 3), ...
    struct('type', 'static', 'center', [55; 23; 3], 'radius', 3)
};
obstacle_scenarios{4} = {
    struct('type', 'static', 'center', [20; -23; 3], 'radius', 3), ...
    struct('type', 'static', 'center', [50; 17; 3], 'radius', 3)
};

%% ======================= 4. Run All Scenarios and Generate Plots =======================
all_states_history = cell(1, 4);
all_inputs_history = cell(1, 4);

for s = 1:4
    fprintf('Running Scenario %d...\n', s);
    obstacles = obstacle_scenarios{s};
    [all_states_history{s}, all_inputs_history{s}] = run_dmpc_scenario(initial_states, leader_trajectory, formation_offsets, obstacles, sim_params, sprintf('Scenario %d', s), false);
end

% Generate combined plot for all scenarios
visualize_all_scenarios(all_states_history, obstacle_scenarios, sim_params);
visualize_all_metrics(all_states_history, all_inputs_history, obstacle_scenarios, leader_trajectory, formation_offsets, sim_params);

%% ======================= Core Simulation Functions (Copied from original) =======================
% ... (The dmpc_controller, compute_cost, nonlinear_constraints, update_dynamics functions
%     remain the same as in the original script. They should be placed here
%     or in separate .m files for better practice) ...

function [all_states, all_inputs] = run_dmpc_scenario(initial_states, leader_trajectory, formation_offsets, obstacles, sim_params, scenario_title, is_event_triggered)
    
    all_states = zeros(6, sim_params.N, sim_params.T_total);
    all_inputs = zeros(3, sim_params.N, sim_params.T_total);
    current_states = initial_states;
    
    dmpc_solvers = cell(1, sim_params.N);
    last_inputs = zeros(3, sim_params.N);
    
    for i = 1:sim_params.N
        dmpc_solvers{i} = @(current_states, k) dmpc_controller(i, current_states, leader_trajectory, formation_offsets, obstacles, k, sim_params);
    end
    
    for k = 1:sim_params.T_total
        u_k = zeros(3, sim_params.N);
        for i = 1:sim_params.N
            if is_event_triggered
                u_k_full = dmpc_solvers{i}(current_states, k);
                
                if norm(u_k_full - last_inputs(:,i)) > sim_params.event_trigger_threshold
                    u_k(:, i) = u_k_full;
                    last_inputs(:, i) = u_k_full;
                else
                    u_k(:, i) = last_inputs(:, i);
                end
            else
                u_k(:, i) = dmpc_solvers{i}(current_states, k);
            end
        end
        
        for i = 1:sim_params.N
            current_states(:, i) = update_dynamics(current_states(:, i), u_k(:, i), sim_params.Ts);
        end
        
        all_states(:, :, k) = current_states;
        all_inputs(:, :, k) = u_k;
    end
end

function u = dmpc_controller(id, current_states, leader_trajectory, formation_offsets, obstacles, k, sim_params)
    u0 = zeros(3 * sim_params.Hc, 1);
    objective_fcn = @(u_seq) compute_cost(u_seq, id, current_states, leader_trajectory, formation_offsets, obstacles, k, sim_params);
    nonlcon_fcn = @(u_seq) nonlinear_constraints(u_seq, id, current_states, obstacles, k, sim_params);
    lb = repmat([-sim_params.max_accel; -sim_params.max_accel; -sim_params.max_accel], sim_params.Hc, 1);
    ub = repmat([sim_params.max_accel; sim_params.max_accel; sim_params.max_accel], sim_params.Hc, 1);
    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
    [u_optimal, ~, exitflag] = fmincon(objective_fcn, u0, [], [], [], [], lb, ub, nonlcon_fcn, options);
    if exitflag <= 0
        u = zeros(3, 1);
    else
        u = u_optimal(1:3);
    end
end

function cost = compute_cost(u_seq, id, current_states, leader_trajectory, formation_offsets, obstacles, k, sim_params)
    cost = 0;
    predicted_states = zeros(6, sim_params.Hp);
    predicted_states(:, 1) = current_states(:, id);
    for h = 1:sim_params.Hp-1
        if h <= sim_params.Hc
            u_h = u_seq(3*(h-1)+1 : 3*h);
        else
            u_h = u_seq(3*(sim_params.Hc-1)+1 : 3*sim_params.Hc);
        end
        predicted_states(:, h+1) = update_dynamics(predicted_states(:, h), u_h, sim_params.Ts);
    end
    
    cost_formation = 0;
    for h = 1:sim_params.Hp
        predicted_pos = predicted_states(1:3, h);
        if id == 1
            target_pos = leader_trajectory(k + h);
        else
            leader_pos_at_future_k = leader_trajectory(k + h);
            target_pos = leader_pos_at_future_k + formation_offsets(:, id-1);
        end
        pos_error = predicted_pos - target_pos;
        cost_formation = cost_formation + pos_error' * sim_params.Q(1:3, 1:3) * pos_error;
    end
    
    cost_avoidance = 0;
    min_safe_dist = 1.5;
    for h = 1:sim_params.Hp
        current_predicted_pos = predicted_states(1:3, h);
        for j = 1:sim_params.N
            if j ~= id
                other_pos = current_states(1:3, j);
                dist = norm(current_predicted_pos - other_pos);
                if dist < min_safe_dist
                    cost_avoidance = cost_avoidance + (min_safe_dist - dist)^2;
                end
            end
        end
        
        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            if strcmp(obs.type, 'dynamic')
                obs_center = obs.center_func(k+h);
            else
                obs_center = obs.center;
            end
            dist = norm(current_predicted_pos - obs_center);
            if dist < (obs.radius + min_safe_dist)
                cost_avoidance = cost_avoidance + ((obs.radius + min_safe_dist) - dist)^2;
            end
        end
    end
    
    cost_control = u_seq' * (kron(eye(sim_params.Hc), sim_params.R)) * u_seq;
    
    cost = sim_params.W_formation * cost_formation + sim_params.W_avoidance * cost_avoidance + cost_control;
end

function [c, ceq] = nonlinear_constraints(u_seq, id, current_states, obstacles, k, sim_params)
    c = [];
    ceq = [];
    predicted_states = zeros(6, sim_params.Hp);
    predicted_states(:, 1) = current_states(:, id);
    
    for h = 1:sim_params.Hp-1
        if h <= sim_params.Hc
             u_h = u_seq(3*(h-1)+1 : 3*h);
        else
            u_h = u_seq(3*(sim_params.Hc-1)+1 : 3*sim_params.Hc);
        end
        predicted_states(:, h+1) = update_dynamics(predicted_states(:, h), u_h, sim_params.Ts);
    end
    
    min_safe_dist = 1.5;
    for h = 1:sim_params.Hp
        vel_magnitude_sq = norm(predicted_states(4:6, h))^2;
        c = [c; vel_magnitude_sq - sim_params.max_vel^2];
        current_pos = predicted_states(1:3, h);
        
        for j = 1:sim_params.N
            if j ~= id
                other_pos = current_states(1:3, j);
                dist_sq = norm(current_pos - other_pos)^2;
                c = [c; min_safe_dist^2 - dist_sq];
            end
        end
        
        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            if strcmp(obs.type, 'dynamic')
                obs_center = obs.center_func(k+h);
            else
                obs_center = obs.center;
            end
            dist_sq = norm(current_pos - obs_center)^2;
            c = [c; (obs.radius + min_safe_dist)^2 - dist_sq];
        end
    end
end

function next_state = update_dynamics(current_state, u, Ts)
    p = current_state(1:3);
    v = current_state(4:6);
    a = u;
    
    next_p = p + v * Ts + 0.5 * a * Ts^2;
    next_v = v + a * Ts;
    
    next_state = [next_p; next_v];
end

function [E11, E12, E2, E3] = calculate_metrics(all_states, all_inputs, obstacles, leader_trajectory, formation_offsets, sim_params)
    T_total = sim_params.T_total;
    N = sim_params.N;

    E11 = zeros(1, T_total);
    E12 = zeros(1, T_total);
    E2 = zeros(1, T_total);
    E3 = zeros(1, T_total);

    for k = 1:T_total
        current_velocities = squeeze(all_states(4:6, :, k));
        current_positions = squeeze(all_states(1:3, :, k));
        
        sum_v_error = 0;
        for i = 1:N
            for j = i+1:N
                sum_v_error = sum_v_error + norm(current_velocities(:, i) - current_velocities(:, j))^2;
            end
        end
        E11(k) = sum_v_error / (N * (N-1) / 2);
        
        leader_pos_at_k = leader_trajectory(k);
        target_positions = [leader_pos_at_k, leader_pos_at_k + formation_offsets];
        pos_error_sum = 0;
        for i = 1:N
            pos_error_sum = pos_error_sum + norm(current_positions(:, i) - target_positions(:, i))^2;
        end
        E12(k) = pos_error_sum / N;
        
        if k > 1
            input_change_sum = 0;
            for i = 1:N
                input_change_sum = input_change_sum + norm(all_inputs(:, i, k) - all_inputs(:, i, k-1))^2;
            end
            E2(k) = input_change_sum / N;
        end
        
        min_dist_uav_obs = Inf;
        min_dist_uav_uav = Inf;
        for i = 1:N
            for j = i+1:N
                dist = norm(current_positions(:, i) - current_positions(:, j));
                if dist < min_dist_uav_uav
                    min_dist_uav_uav = dist;
                end
            end
            
            for obs_idx = 1:length(obstacles)
                obs = obstacles{obs_idx};
                if strcmp(obs.type, 'dynamic')
                    obs_center = obs.center_func(k);
                else
                    obs_center = obs.center;
                end
                dist = norm(current_positions(:, i) - obs_center) - obs.radius;
                if dist < min_dist_uav_obs
                    min_dist_uav_obs = dist;
                end
            end
        end
        E3(k) = min(min_dist_uav_obs, min_dist_uav_uav);
    end
end
% 
% ... (The rest of the core functions should be placed here) ...

%% ======================= 5. New Visualization Functions for SCI Standard =======================

function visualize_all_scenarios(all_states_history, obstacle_scenarios, sim_params)
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
    set(groot, 'defaultTextInterpreter', 'latex');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultAxesFontSize', 12);
    set(groot, 'defaultLineLineWidth', 1.5);
    
    figure('Name', 'Multi-Scenario Trajectory Plot', 'Position', [100, 100, 1000, 800]);
    
    num_uavs = sim_params.N;
    colors = lines(num_uavs);
    
    for s = 1:4
        h_ax = subplot(2, 2, s);
        hold on;
        
        % Plot UAV trajectories
        for i = 1:num_uavs
            plot3(squeeze(all_states_history{s}(1, i, :)), ...
                  squeeze(all_states_history{s}(2, i, :)), ...
                  squeeze(all_states_history{s}(3, i, :)), ...
                  'Color', colors(i,:));
        end
        
        % Plot Obstacles
        obstacles = obstacle_scenarios{s};
        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            obs_center = obs.center;
            [x, y, z] = sphere;
            x = x * obs.radius + obs_center(1);
            y = y * obs.radius + obs_center(2);
            z = z * obs.radius + obs_center(3);
            surf(h_ax, x, y, z, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineStyle', 'none');
            
            % Add a vertical cylinder to represent the obstacle
            [X, Y, Z] = cylinder(obs.radius, 50);
            Z = Z * 15; % Assuming obstacles go up to z=15
            Z = Z + obs_center(3);
            surf(h_ax, X + obs_center(1), Y + obs_center(2), Z, 'FaceColor', [1, 0.5, 0], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineStyle', 'none');
        end
        
        % Set plot properties
        title(h_ax, ['(', char('a' + s - 1), ') Scenario ', num2str(s)]);
        xlabel(h_ax, '$X$ (m)');
        ylabel(h_ax, '$Y$ (m)');
        zlabel(h_ax, '$Z$ (m)');
        grid on;
        box on;
        axis equal;
        axis(h_ax, [-10 65 -30 30 0 15]);
        view(h_ax, 45, 30);
    end
    
    sgtitle('Three-dimensional formation trajectories in different scenarios', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Create a common legend for the entire figure
    h_temp = figure('Visible', 'off');
    hold on;
    for i = 1:num_uavs
        plot(0, 0, 'Color', colors(i,:), 'DisplayName', sprintf('UAV %d', i));
    end
    legend_obj = legend('Location', 'best');
    set(legend_obj, 'Interpreter', 'latex', 'FontSize', 12);
    
    
    % Get handles to the main subplots to move the legend there
    h_ax1 = subplot(2,2,1);
    lgd = legend(h_ax1, 'UAV 1', 'UAV 2', 'UAV 3', 'UAV 4', 'UAV 5', 'Location', 'northwest');
    set(lgd, 'Interpreter', 'latex');
    
    % Close the temporary figure
    close(h_temp);
    
    set(gcf, 'PaperPositionMode', 'auto');
    print(gcf, 'SCI_trajectories_plot.png', '-dpng', '-r300');
end

function visualize_all_metrics(all_states_history, all_inputs_history, obstacle_scenarios, leader_trajectory, formation_offsets, sim_params)
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
    set(groot, 'defaultTextInterpreter', 'latex');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultAxesFontSize', 12);
    set(groot, 'defaultLineLineWidth', 1.5);
    
    figure('Name', 'Multi-Scenario Performance Metrics', 'Position', [100, 100, 1000, 800]);
    
    metrics = cell(4, 4); % rows: scenarios, cols: E11, E12, E2, E3
    metric_names = {'Velocity Consensus', 'Position Matching', 'Input Change', 'Obstacle Safety'};
    metric_labels = {'$E_{1,1}(k)$', '$E_{1,2}(k)$', '$E_2(k)$', '$E_3(k)$'};
    
    for s = 1:4
        [E11, E12, E2, E3] = calculate_metrics(all_states_history{s}, all_inputs_history{s}, obstacle_scenarios{s}, leader_trajectory, formation_offsets, sim_params);
        metrics{s, 1} = E11;
        metrics{s, 2} = E12;
        metrics{s, 3} = E2;
        metrics{s, 4} = E3;
    end
    
    for m = 1:4 % loop through metrics
        subplot(2, 2, m);
        hold on;
        for s = 1:4 % loop through scenarios
            plot(1:sim_params.T_total, metrics{s, m}, 'DisplayName', sprintf('Scenario %d', s));
        end
        title(metric_names{m});
        xlabel('Time Step (k)');
        ylabel(metric_labels{m});
        grid on;
        legend('Location', 'best');
        hold off;
    end
    
    sgtitle('Performance Metrics Comparison for All Scenarios', 'FontSize', 14, 'FontWeight', 'bold');
    
    set(gcf, 'PaperPositionMode', 'auto');
    print(gcf, 'SCI_metrics_plot.png', '-dpng', '-r300');
end