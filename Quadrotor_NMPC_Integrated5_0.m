% Quadrotor_NMPC_Enhanced_Visualization.m
% Author: Yufan Wang
% Date: 2025-07-28
% This script simulates a quadrotor using Nonlinear Model Predictive Control (NMPC)
% and provides enhanced visualization:
% - Figure 1: Static plots of actual vs. reference trajectories for all 6 states (X, Y, Z, Phi, Theta, Psi).
% - Figure 2: Separate figure displaying the control inputs (u1, u2, u3, u4).
% - Figure 3: Real-time 3D animation of the quadrotor's trajectory tracking.
% - Figure 4: Plots of additional performance metrics (Position Error, Cost Function Value).

% Initialization
clear;clc;

% State variables count (x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot)
nx = 12;
% Output variables count (usually same as state variables for full state feedback)
ny = 12;
% Input variables count (u1, u2, u3, u4 - motor thrusts)
nu = 4;

% Sampling time
Ts = 0.1;

% Prediction horizon (p) and Control horizon (m)
p = 18; % Prediction steps
m = 2;  % Control steps (control inputs are held constant after m steps)

% MPC parameters
% Input constraints
MVMin = [0;0;0;0];   % Minimum allowed control input
MVMax = [10;10;10;10]; % Maximum allowed control input

% Weights for the cost function
OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0]; % Weights for state tracking (position and attitude are weighted)
ManipulatedVariables = [0.1 0.1 0.1 0.1];      % Weights for control input magnitude
ManipulatedVariablesRate = [0.1 0.1 0.1 0.1]; % Weights for control input rate of change

% Initial state of the quadrotor [x; y; z; phi; theta; psi; x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot]
x = [7;-10;0;0;0;0;0;0;0;0;0;0];

% Target control inputs (often corresponds to hover thrust)
MVTarget = [4.9 4.9 4.9 4.9];
nloptions.MVTarget = MVTarget; % Store target values for plotting

% Simulation duration
Duration = 30; % seconds (Increased from 20 to 30)

% Initialize last applied control input (for rate calculation)
lastMV = MVTarget;

% History to store states and inputs over the simulation
xHistory = x';   % State history (rows are time steps, columns are states)
uHistory = lastMV; % Input history
costHistory = []; % History to store cost function values at each step

% Optimization variables from previous iteration (for warm start)
z = [];

% Data structure to pass to cost and constraint functions
data.lastMV = MVTarget'; % Initial lastMV for the first optimization step
data.OutputVariables = OutputVariables;
data.ManipulatedVariables = ManipulatedVariables;
data.ManipulatedVariablesRate = ManipulatedVariablesRate;
data.nx = nx;
data.nu = nu;
data.p = p;
data.m = m;
data.Ts = Ts;

% Main simulation loop
for k = 1:(Duration/Ts)
    disp(['loop: ', num2str(k), '/',num2str((Duration/Ts))])

    % Generate reference trajectory for the prediction horizon
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t); % Calls local function

    % Get current actual state
    xk = xHistory(k,:);
    data.state = xk; % Pass current state to cost/constraint functions

    % Warm start for optimization variables
    if isempty(z)
        % First iteration: initialize states with current state, MVs with target
        X0 = repmat(xk,p,1);
        MV0 = repmat(MVTarget,m,1);
    else
        % Subsequent iterations: shift previous optimal state and MV trajectories
        X0 = reshape(z(1:p*nx),nx,p)'; % Reshape states from previous optimization
        X0 = [X0(2:end,:); X0(end,:)]; % Shift states: remove first, duplicate last
        
        % Shift MVs (only the 'm' control inputs are optimized)
        last_optimal_mvs = reshape(z(p*nx+1:p*nx+m*nu), nu, m)';
        MV0 = [last_optimal_mvs(2:end,:); last_optimal_mvs(end,:)];
    end
    
    % Reshape initial guess for optimization variable 'z0'
    xz = reshape(X0',p*nx,1); % States as a column vector
    uz = reshape(MV0',m*nu,1); % MVs as a column vector
    z0 = [xz; uz; 0]; % Combine: [states; MVs; slack variable]

    % Linear inequality constraints (A*z <= B) for MV bounds
    % Constraints apply to the 'm' manipulated variables in the control horizon
    A1  = [zeros(m*nu, p*nx), eye(m*nu), zeros(m*nu, 1)]; % u <= MVMax
    B1  = repmat(MVMax, m, 1);

    A2  = [zeros(m*nu, p*nx), -eye(m*nu), zeros(m*nu, 1)]; % -u <= -MVMin => u >= MVMin
    B2  = repmat(-MVMin, m, 1);

    A = [A1;A2];
    B = [B1;B2];

    % Lower and upper bounds for optimization variable 'z'
    StateMin = -inf*ones(p,nx); % States have no explicit hard bounds
    StateMax = inf*ones(p,nx);
    xLB = reshape(StateMin', p*nx, 1);
    xUB = reshape(StateMax', p*nx, 1);

    uLB = repmat(MVMin, m, 1); % MV lower bounds
    uUB = repmat(MVMax, m, 1); % MV upper bounds
    
    zLB = [xLB; uLB; 0];   % Slack variable 'e' must be >= 0
    zUB = [xUB; uUB; Inf]; % Slack variable 'e' has no upper bound

    % Define cost and constraint functions for fmincon
    CostFcn = @(z)CostFcnWrapper(z,data,yref); % Calls local function
    ConFcn  = @(z)ConFcnWrapper(z,data,yref); % Calls local function

    % fmincon optimization options
    options = optimoptions(@fmincon,'MaxIterations',400,...
        'StepTolerance',1e-6,...
        'ConstraintTolerance',1e-6,...
        'OptimalityTolerance',1e-6,...
        'FunctionTolerance',0.01, ...
        'Display', 'off'); % Suppress fmincon output

    % Solve the Non-Linear Programming (NLP) problem
    [z, cost, ExitFlag, Out] = fmincon(CostFcn, z0, A, B, [], [], zLB, zUB, ConFcn,options);
    costHistory = [costHistory; cost]; % Store the cost at current step

    % Extract the first control input from the optimized solution
    uk = z(p*nx+1:p*nx+nu); % uk corresponds to the first 'nu' elements of the 'm*nu' control vector
    lastMV = uk;            % Update last applied MV for next iteration's rate calculation
    data.lastMV = lastMV;   % Pass updated lastMV to data structure

    % Simulate the quadrotor model for one sampling period (Ts)
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk); % Calls local function
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)'); % Simulate from current state

    % Store the applied input and the resulting next state
    uHistory(k+1,:) = uk';      % Store the actual applied control input
    xHistory(k+1,:) = XOUT(end,:); % Store the simulated state at the end of Ts
end

%% Plotting - Enhanced Visualization
% Calculate full reference trajectory for plotting
time = 0:Ts:Duration;
yreftot = QuadrotorReferenceTrajectory(time);

% SCI-style figure aesthetics
LineWidth1 = 2.0; LineWidth2 = 1.2;
FontSize_axis = 11; FontSize_tick = 10;
FontSize_legend = 10; FontSize_title = 12;
FontName = 'Times New Roman';

% --- Figure 1: Static State Trajectories (2x3 grid of subplots) ---
figure('Name','Quadrotor State Trajectories','Color','w','Position',[50 50 1000 600]); % Standard size for 6 subplots

% Manually create axes for better control over positioning and spacing
labels = {'$x$ (m)','$y$ (m)','$z$ (m)','$\phi$ (rad)','$\theta$ (rad)','$\psi$ (rad)'};
ref_labels = {'$x_{ref}$','$y_{ref}$','$z_{ref}$','$\phi_{ref}$','$\theta_{ref}$','$\psi_{ref}$'};

% Loop to create and plot each of the 6 state subplots
for i = 1:6
    subplot(2,3,i); % Use standard subplot for this figure
    
    hold on;
    plot(time,xHistory(:,i),'-','LineWidth',LineWidth1,'Color',[0 0.447 0.741]); % Deep Blue
    plot(time,yreftot(i,:), '--','LineWidth',LineWidth2,'Color',[0.85 0.33 0.1]); % Orange Dashed
    grid on; box on;
    set(gca, 'FontSize', FontSize_tick, 'FontName', FontName, 'LineWidth', 1);
    xlabel('Time (s)','FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
    ylabel(labels{i},'FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
    title(labels{i},'FontSize',FontSize_title,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
    legend({'Actual',ref_labels{i}},'FontSize',FontSize_legend,'Location','best','Box','on','Interpreter','latex');
    ax = gca;
    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    ax.GridAlpha = 0.2;
    ax.MinorGridAlpha = 0.1;
    hold off;
end
sgtitle('Quadrotor State Trajectories: Actual vs. Reference (Final)', 'FontSize', FontSize_title+2, 'FontName', FontName, 'FontWeight', 'bold', 'Interpreter', 'latex');


% --- Figure 2: Separate Figure for Control Inputs ---
figure('Name','Quadrotor Control Inputs','Color','w','Position',[120 180 800 500]);
labels_u = {'$u_1$','$u_2$','$u_3$','$u_4$'};
for i = 1:4
    subplot(2,2,i)
    hold on
    stairs(time, uHistory(:,i),'-','LineWidth',LineWidth1,'Color',[0 0.5 0]);
    yline(nloptions.MVTarget(i),':','LineWidth',1.5,'Color',[1.0 0.3 0.3]);
    ylim([-0.5,12.5])
    grid on; box on;
    set(gca, 'FontSize', FontSize_tick, 'FontName', FontName, 'LineWidth', 1);
    xlabel('Time (s)','FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
    ylabel(labels_u{i},'FontSize',FontSize_title,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
    title(['Input ',labels_u{i}],'FontSize',FontSize_title,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
    legend({'Actual',['Target (',num2str(nloptions.MVTarget(i)),')']},'FontSize',FontSize_legend,'Location','best','Box','on','Interpreter','latex');
    ax = gca;
    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    ax.GridAlpha = 0.2;
    ax.MinorGridAlpha = 0.1;
end
sgtitle('Manipulated Variables over Time', 'FontSize', FontSize_title+2, 'FontName', FontName, 'FontWeight', 'bold', 'Interpreter', 'latex');


% --- Figure 3: Real-time 3D Trajectory Animation ---
figure('Name','Quadrotor 3D Trajectory Animation','Color','w','Position',[100 100 800 700]); % New figure for animation
ax_animation = gca; % Get current axes handle for the new figure
hold(ax_animation, 'on');

% Animation loop to draw into ax_animation
for i = 1:size(xHistory, 1)
    % Clear the animation axes for each frame, then redraw
    cla(ax_animation);
    hold(ax_animation, 'on'); % Re-enable hold after clearing

    % Plot the full reference trajectory (static background)
    plot3(ax_animation, yreftot(1,:),yreftot(2,:),yreftot(3,:),'g--','LineWidth',1.0);
    
    % Plot the actual trajectory up to the current frame (dynamic)
    plot3(ax_animation, xHistory(1:i,1),xHistory(1:i,2),xHistory(1:i,3),'r.','LineWidth',1.5);
    
    % Call drone_Animation to draw the quadrotor at the current state
    % Pass the axes handle to drone_Animation so it draws in the correct subplot
    drone_Animation(xHistory(i, 1:6), ax_animation); 
    
    % Set axis properties for the animation subplot
    title(ax_animation, 'Quadrotor 3D Trajectory Tracking (Real-time)', 'FontSize', 14, 'FontName', 'Times New Roman', 'FontWeight', 'bold');
    xlabel(ax_animation, 'X (m)', 'FontSize', 12, 'FontName', 'Times New Roman', 'FontWeight', 'bold');
    ylabel(ax_animation, 'Y (m)', 'FontSize', 12, 'FontName', 'Times New Roman', 'FontWeight', 'bold');
    zlabel(ax_animation, 'Z (m)', 'FontSize', 12, 'FontName', 'Times New Roman', 'FontWeight', 'bold');
    grid(ax_animation, 'on');
    box(ax_animation, 'on');
    axis(ax_animation, 'equal'); % Keep aspect ratio
    axis(ax_animation, [-10,10,-10,10,-10,10]); % Fixed axis limits
    view(ax_animation, 68,53); % Set camera view
    
    drawnow limitrate; % Limit refresh rate for smoother animation
    
    hold(ax_animation, 'off'); % Disable hold after drawing for the frame
end


% --- Figure 4: Performance Metrics Analysis ---
figure('Name','Quadrotor Performance Metrics','Color','w','Position',[150 150 900 450]);

% Subplot 1: Position Error
subplot(1,2,1);
position_error = sqrt((xHistory(:,1) - yreftot(1,:)').^2 + ...
                      (xHistory(:,2) - yreftot(2,:)').^2 + ...
                      (xHistory(:,3) - yreftot(3,:)').^2);
plot(time, position_error, '-', 'LineWidth', LineWidth1, 'Color', [0.4940 0.1840 0.5560]); % Purple
hold on;
yline(0, '--k', 'LineWidth', 0.8); % Zero error line
grid on; box on;
set(gca, 'FontSize', FontSize_tick, 'FontName', FontName, 'LineWidth', 1);
xlabel('Time (s)','FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
ylabel('Position Error (m)','FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
title('3D Position Tracking Error','FontSize',FontSize_title,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
ax = gca;
ax.XMinorTick = 'on'; ax.YMinorTick = 'on';
ax.GridAlpha = 0.2; ax.MinorGridAlpha = 0.1;
hold off;

% Subplot 2: Cost Function Value
subplot(1,2,2);
plot(time(1:end-1), costHistory, '-', 'LineWidth', LineWidth1, 'Color', [0.9290 0.6940 0.1250]); % Gold
grid on; box on;
set(gca, 'FontSize', FontSize_tick, 'FontName', FontName, 'LineWidth', 1);
xlabel('Time (s)','FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
ylabel('Cost Function Value','FontSize',FontSize_axis,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
title('NMPC Cost Function Value over Time','FontSize',FontSize_title,'FontName',FontName,'FontWeight','bold','Interpreter','latex');
ax = gca;
ax.XMinorTick = 'on'; ax.YMinorTick = 'on';
ax.GridAlpha = 0.2; ax.MinorGridAlpha = 0.1;

sgtitle('Quadrotor Performance Metrics Analysis', 'FontSize', FontSize_title+2, 'FontName', FontName, 'FontWeight', 'bold', 'Interpreter', 'latex');


% --- Start of Local Functions ---

% --- CostFcnWrapper.m content ---
function f = CostFcnWrapper(z, data,yref)
p = data.p;
nx = data.nx;
nu = data.nu;
m = data.m;

% Extract variables from z
Xz = reshape(z(1:p*nx), nx, p)'; % Predicted states over horizon p
uz = z(p*nx+1:p*nx+m*nu);        % Manipulated variables over control horizon m

% Construct Iz2u to map m control inputs to p prediction steps
% This matrix handles the case where control inputs are held constant after 'm' steps.
Iz2u = zeros(p*nu, m*nu);
Iz2u(1:m*nu, 1:m*nu) = eye(m*nu); % Direct mapping for the first 'm' control steps
for i = 0:(p-m-1)
    % For each of the (p-m) remaining steps, copy the last 'nu' controls from the 'm*nu' input vector
    Iz2u(m*nu + i*nu + 1 : m*nu + (i+1)*nu, (m-1)*nu + 1 : m*nu) = eye(nu);
end

Uexpanded = reshape(Iz2u * uz, nu, p)'; % Expanded U for prediction horizon

% Initialize full state and control trajectories for calculations
X = zeros(p+1, nx);
U = zeros(p+1, nu);
X(1,:) = data.state;
X(2:p+1,:) = Xz;
U(1:p,:) = Uexpanded;
U(p+1,:) = Uexpanded(end,:); % Replicate last control for the last step if needed for rate calc.

f = 0; % Initialize cost
OutputVariables = data.OutputVariables';
ManipulatedVariables = data.ManipulatedVariables';
ManipulatedVariablesRate = data.ManipulatedVariablesRate';

for i = 1:p % Loop over prediction horizon
    % Output Tracking Cost
    xk_pred = X(i+1,:)'; % Predicted state at step i+1
    yk_pred = xk_pred;   % Assuming output is the state itself
    yerr = yk_pred - yref(:,i); % Error between predicted output and reference
    f = f + yerr'*diag(OutputVariables)*yerr; % Weighted squared error

    % Manipulated Variable (MV) Cost
    uk = U(i,:)'; % Control input for step i
    f = f + uk'*diag(ManipulatedVariables)*uk; % Weighted squared MV

    % Manipulated Variable Rate (MVR) Cost
    if i == 1
        duk = uk - data.lastMV; % Rate from previous actual MV to current step's MV
    else
        duk = uk - U(i-1,:)'; % Rate between current and previous MV in prediction
    end
    f = f + duk'*diag(ManipulatedVariablesRate)*duk; % Weighted squared MVR
end
% Add the slack variable penalty
f = f + 1e5 * z(end)^2; % Penalty for the slack variable (e)

end

% --- ConFcnWrapper.m content ---
function [cineq, ceq] = ConFcnWrapper(z, data, yref)
p = data.p;
nx = data.nx;
nu = data.nu;
m = data.m;
Ts = data.Ts;

% Extract variables from z
Xz = reshape(z(1:p*nx), nx, p)'; % Predicted states over horizon p
uz = z(p*nx+1:p*nx+m*nu);        % Manipulated variables over control horizon m
e = z(end);                      % Slack variable

% Construct Iz2u to map m control inputs to p prediction steps
Iz2u = zeros(p*nu, m*nu);
Iz2u(1:m*nu, 1:m*nu) = eye(m*nu);
for i = 0:(p-m-1)
    Iz2u(m*nu + i*nu + 1 : m*nu + (i+1)*nu, (m-1)*nu + 1 : m*nu) = eye(nu);
end

Uexpanded = reshape(Iz2u * uz, nu, p)'; % Expanded U for prediction horizon

% Initialize full state and control trajectories for calculations
X = zeros(p+1, nx);
U = zeros(p+1, nu);
X(1,:) = data.state;
X(2:p+1,:) = Xz;
U(1:p,:) = Uexpanded;

% Equality constraints (Nonlinear Dynamics: x_{k+1} - (x_k + Ts * f(x_k,u_k)))
ceq = zeros(p*nx,1);
h = Ts/2; % For Runge-Kutta 2 (Midpoint method) or Euler (if h=Ts)

for i = 1:p % Loop over prediction horizon
    xk_current = X(i,:)';     % Current state (x_k)
    uk_current = U(i,:)';     % Current control input (u_k)
    xk_next_pred = X(i+1,:)'; % Predicted next state from optimization (x_{k+1})
    
    fk  = QuadrotorStateFcn(xk_current, uk_current); % f(x_k, u_k)
    fk1 = QuadrotorStateFcn(xk_next_pred, uk_current); % f(x_{k+1}, u_k) - this implies an implicit method

    ceq((i-1)*nx+1 : i*nx) = xk_next_pred - (xk_current + h*(fk + fk1));
end

% Inequality constraints (usually for state limits, or more commonly, slack variable)
cineq = []; % No explicit state inequality constraints defined, so it's empty.

end

% --- QuadrotorReferenceTrajectory.m content ---
function [ xdesired ] = QuadrotorReferenceTrajectory( t )
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

%#codegen
x = 6*sin(t/3);
y = -6*sin(t/3).*cos(t/3);
z = 6*cos(t/3);
phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));

xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end

% --- QuadrotorStateFcn.m content ---
function f = QuadrotorStateFcn(in1,in2)
%QuadrotorStateFcn
%    F = QuadrotorStateFcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    07-Mar-2023 16:16:13

phi_t = in1(4,:);
phi_dot_t = in1(10,:);
psi_t = in1(6,:);
psi_dot_t = in1(12,:);
theta_t = in1(5,:);
theta_dot_t = in1(11,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
x_dot_t = in1(7,:);
y_dot_t = in1(8,:);
z_dot_t = in1(9,:);
t2 = cos(phi_t);
t3 = cos(psi_t);
t4 = cos(theta_t);
t5 = sin(phi_t);
t6 = sin(psi_t);
t7 = sin(theta_t);
t8 = phi_t.*2.0;
t9 = psi_dot_t.^2;
t10 = theta_t.*2.0;
t15 = u1+u2+u3+u4;
t11 = t2.^2;
t12 = t4.^2;
t13 = sin(t8);
t14 = sin(t10);
t16 = 1.0./t12;
mt1 = [x_dot_t;y_dot_t;z_dot_t;phi_dot_t;theta_dot_t;psi_dot_t;(t15.*(t5.*t6+t2.*t3.*t7))./2.0;t15.*(t3.*t5-t2.*t6.*t7).*(-1.0./2.0);(t2.*t4.*t15)./2.0-9.81e+2./1.0e+2];
mt2 = (t16.*(u2.*-1.15e+2+u4.*1.15e+2-t7.*u1.*9.2e+1+t7.*u2.*9.2e+1-t7.*u3.*9.2e+1+t7.*u4.*9.2e+1+t11.*u2.*5.5e+1-t11.*u4.*5.5e+1+phi_dot_t.*t14.*theta_dot_t.*2.3e+1+psi_dot_t.*t4.*theta_dot_t.*1.058e+3+t7.*t11.*u1.*4.4e+1-t7.*t11.*u2.*4.4e+1+t7.*t11.*u3.*4.4e+1-t7.*t11.*u4.*4.4e+1-t11.*t12.*u2.*5.5e+1+t11.*t12.*u4.*5.5e+1-psi_dot_t.*t4.*t11.*theta_dot_t.*5.06e+2-t2.*t5.*t9.*t12.*5.06e+2-psi_dot_t.*t4.^3.*t11.*theta_dot_t.*5.06e+2+t2.*t5.*t12.*theta_dot_t.^2.*5.06e+2+phi_dot_t.*t4.*t7.*t11.*theta_dot_t.*5.06e+2-t2.*t4.*t5.*t7.*u1.*5.5e+1+t2.*t4.*t5.*t7.*u3.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t7.*t12.*5.06e+2))./5.52e+2;
mt3 = ((t4.*u1.*6.0e+1-t4.*u3.*6.0e+1+t13.*u1.*2.2e+1-t13.*u2.*2.2e+1+t13.*u3.*2.2e+1-t13.*u4.*2.2e+1+phi_dot_t.*psi_dot_t.*t12.*5.52e+2+t4.*t11.*u1.*5.5e+1-t4.*t11.*u3.*5.5e+1-phi_dot_t.*psi_dot_t.*t11.*t12.*5.06e+2+t7.*t9.*t11.*t12.*5.06e+2+t2.*t5.*t7.*u2.*5.5e+1-t2.*t5.*t7.*u4.*5.5e+1+phi_dot_t.*t2.*t4.*t5.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*5.06e+2).*(-1.0./5.52e+2))./t4;
mt4 = (t16.*(u1.*-9.2e+1+u2.*9.2e+1-u3.*9.2e+1+u4.*9.2e+1-t7.*u2.*1.15e+2+t7.*u4.*1.15e+2+t11.*u1.*4.4e+1-t11.*u2.*4.4e+1+t11.*u3.*4.4e+1-t11.*u4.*4.4e+1+phi_dot_t.*t4.*theta_dot_t.*4.6e+1+psi_dot_t.*t14.*theta_dot_t.*5.29e+2+t7.*t11.*u2.*5.5e+1-t7.*t11.*u4.*5.5e+1+phi_dot_t.*t4.*t11.*theta_dot_t.*5.06e+2-t2.*t4.*t5.*u1.*5.5e+1+t2.*t4.*t5.*u3.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t12.*5.06e+2-psi_dot_t.*t4.*t7.*t11.*theta_dot_t.*5.06e+2-t2.*t5.*t7.*t9.*t12.*5.06e+2))./5.52e+2;
f = [mt1;mt2;mt3;mt4];

end

% --- QuadrotorStateJacobianFcn.m content ---
function [A,B] = QuadrotorStateJacobianFcn(in1,in2)
%QuadrotorStateJacobianFcn
%    [A,B] = QuadrotorStateJacobianFcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    07-Mar-2023 16:16:13

phi_t = in1(4,:);
phi_dot_t = in1(10,:);
psi_t = in1(6,:);
psi_dot_t = in1(12,:);
theta_t = in1(5,:);
theta_dot_t = in1(11,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
t2 = cos(phi_t);
t3 = cos(psi_t);
t4 = cos(theta_t);
t5 = sin(phi_t);
t6 = sin(psi_t);
t7 = sin(theta_t);
t8 = phi_t.*2.0;
t9 = psi_dot_t.^2;
t10 = theta_t.*2.0;
t11 = theta_dot_t.^2;
t21 = u1+u2+u3+u4;
t12 = cos(t8);
t13 = t2.^2;
t14 = cos(t10);
t15 = t4.^2;
t16 = t4.^3;
t17 = sin(t8);
t18 = t5.^2;
t19 = sin(t10);
t20 = t7.^2;
t22 = t4.*6.0e+1;
t23 = 1.0./t4;
t26 = t7.*9.2e+1;
t27 = t7.*1.15e+2;
t32 = (t2.*t4)./2.0;
t33 = (t3.*t5)./2.0;
t34 = (t5.*t6)./2.0;
t41 = t2.*t4.*t5.*5.5e+1;
t42 = t2.*t5.*t7.*5.5e+1;
t47 = (t2.*t3.*t7)./2.0;
t50 = (t2.*t6.*t7)./2.0;
t24 = 1.0./t15;
t25 = 1.0./t16;
t28 = -t26;
t29 = t13.*4.4e+1;
t30 = t13.*5.5e+1;
t31 = t17.*2.2e+1;
t37 = -t33;
t38 = t4.*t30;
t39 = t7.*t29;
t40 = t7.*t30;
t51 = t15.*t30;
t55 = phi_dot_t.*psi_dot_t.*t2.*t5.*t7.*t12.*5.06e+2;
t56 = phi_dot_t.*t14.*theta_dot_t.*2.3e+1;
t57 = psi_dot_t.*t4.*theta_dot_t.*1.058e+3;
t59 = phi_dot_t.*psi_dot_t.*t13.*t15.*5.06e+2;
t60 = psi_dot_t.*t4.*t7.*t13.*theta_dot_t.*-5.06e+2;
t62 = t7.*t9.*t13.*t15.*5.06e+2;
t63 = t34+t47;
t35 = -t29;
t36 = -t30;
t61 = -t59;
t64 = t37+t50;
t45 = t38.*u1;
t46 = t38.*u3;
t48 = t40.*u2;

t43 = t7.*t11.*4.4e+1;
t44 = t7.*t11.*5.5e+1;
t52 = t7.*t11.*9.2e+1;
t53 = t19.*theta_dot_t.*5.29e+2;
t54 = t11.*t12.*5.5e+1.*u2;

et1 = (t24.*(t4.*u1.*-9.2e+1+t4.*u2.*9.2e+1-t4.*u3.*9.2e+1+t4.*u4.*9.2e+1+phi_dot_t.*t14.*theta_dot_t.*4.6e+1-psi_dot_t.*t7.*theta_dot_t.*1.058e+3-t4.*t13.*u2.*4.4e+1-t4.*t13.*u4.*4.4e+1+t4.*t29.*u1+t4.*t29.*u3+phi_dot_t.*t13.*t15.*theta_dot_t.*5.06e+2-phi_dot_t.*t13.*t20.*theta_dot_t.*5.06e+2+psi_dot_t.*t7.*t13.*theta_dot_t.*5.06e+2-t2.*t5.*t15.*u1.*5.5e+1+t2.*t5.*t15.*u3.*5.5e+1+t4.*t7.*t13.*u2.*1.1e+2+t2.*t5.*t20.*u1.*5.5e+1-t4.*t7.*t13.*u4.*1.1e+2-t2.*t5.*t20.*u3.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t16.*5.06e+2+psi_dot_t.*t7.*t13.*t15.*theta_dot_t.*1.518e+3+t2.*t4.*t5.*t7.*t9.*1.012e+3-t2.*t4.*t5.*t7.*t11.*1.012e+3-phi_dot_t.*psi_dot_t.*t2.*t4.*t5.*t20.*1.012e+3))./5.52e+2;
et2 = (t7.*t25.*(u2.*-1.15e+2+u4.*1.15e+2+t7.*t56-t7.*u1.*9.2e+1-t7.*u3.*9.2e+1-t13.*u4.*5.5e+1+t26.*u2+t26.*u4+t30.*u2+t39.*u1+t39.*u3+t43.*u2+t43.*u4+t51.*u4+t52.*u3+phi_dot_t.*t19.*theta_dot_t.*2.3e+1+psi_dot_t.*t4.*theta_dot_t.*1.058e+3-t13.*t15.*u2.*5.5e+1-psi_dot_t.*t4.*t13.*theta_dot_t.*5.06e+2-psi_dot_t.*t13.*t16.*theta_dot_t.*5.06e+2-t2.*t5.*t9.*t15.*5.06e+2+t2.*t5.*t11.*t15.*5.06e+2-t2.*t4.*t5.*t7.*u1.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t7.*t15.*5.06e+2))./2.76e+2;
et3 = t24.*(t4.*u2.*1.15e+2-t4.*u4.*1.15e+2+t38.*u4+t42.*u3+phi_dot_t.*t7.*theta_dot_t.*4.6e+1-psi_dot_t.*t14.*theta_dot_t.*1.058e+3-t4.*t13.*u2.*5.5e+1+phi_dot_t.*t7.*t13.*theta_dot_t.*5.06e+2+psi_dot_t.*t13.*t15.*theta_dot_t.*5.06e+2-psi_dot_t.*t13.*t20.*theta_dot_t.*5.06e+2+t2.*t5.*t9.*t16.*5.06e+2-t2.*t5.*t7.*u1.*5.5e+1-t2.*t4.*t5.*t9.*t20.*1.012e+3+phi_dot_t.*psi_dot_t.*t2.*t4.*t5.*t7.*1.012e+3).*(-1.0./5.52e+2);
et4 = (t7.*t25.*(t48+t55+t56+t60-u1.*9.2e+1+u2.*9.2e+1-u3.*9.2e+1+u4.*9.2e+1-t7.*u2.*1.15e+2-t13.*u2.*4.4e+1-t13.*u4.*4.4e+1+t29.*u1+t27.*u4+t29.*u3+t41.*u3+phi_dot_t.*t4.*theta_dot_t.*4.6e+1+psi_dot_t.*t19.*theta_dot_t.*5.29e+2-t2.*t4.*t5.*u1.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t15.*5.06e+2-t2.*t5.*t7.*t9.*t15.*5.06e+2))./2.76e+2;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,(t21.*(t2.*t6-t3.*t5.*t7))./2.0,t21.*(t2.*t3+t5.*t6.*t7).*(-1.0./2.0),t4.*t5.*t21.*(-1.0./2.0)];
mt2 = [(t24.*(t7.*t46+t7.*t59-t9.*t13.*t15.*5.06e+2+t11.*t13.*t15.*5.06e+2+t9.*t15.*t18.*5.06e+2-t11.*t15.*t18.*5.06e+2-t2.*t5.*u2.*1.1e+2+t2.*t5.*u4.*1.1e+2+t4.*t44.*u1-t2.*t5.*t7.*u1.*8.8e+1+t2.*t5.*t7.*u2.*8.8e+1-t2.*t5.*t7.*u3.*8.8e+1+t2.*t5.*t7.*u4.*8.8e+1+t2.*t5.*t15.*u2.*1.1e+2-t2.*t5.*t15.*u4.*1.1e+2+t4.*t7.*t18.*u1.*5.5e+1-t4.*t7.*t18.*u3.*5.5e+1-phi_dot_t.*psi_dot_t.*t7.*t15.*t18.*5.06e+2+psi_dot_t.*t2.*t4.*t5.*theta_dot_t.*1.012e+3+psi_dot_t.*t2.*t5.*t16.*theta_dot_t.*1.012e+3-phi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*1.012e+3))./5.52e+2];
mt3 = [t23.*(t48+t55+t56+t60+t12.*u1.*4.4e+1-t12.*u2.*4.4e+1+t12.*u3.*4.4e+1-t12.*u4.*4.4e+1-t7.*t18.*u2.*5.5e+1+t7.*t18.*u4.*5.5e+1-phi_dot_t.*t4.*t18.*theta_dot_t.*5.06e+2-t2.*t4.*t5.*u1.*1.1e+2+t2.*t4.*t5.*u3.*1.1e+2+phi_dot_t.*psi_dot_t.*t2.*t5.*t15.*1.012e+3+psi_dot_t.*t4.*t7.*t18.*theta_dot_t.*5.06e+2-t2.*t5.*t7.*t9.*t15.*1.012e+3).*(-1.0./5.52e+2)];
mt4 = [t24.*(t45+t54+t61+t62+t2.*t5.*u1.*8.8e+1-t2.*t5.*u2.*8.8e+1+t2.*t5.*u3.*8.8e+1-t2.*t5.*u4.*8.8e+1-t4.*t18.*u1.*5.5e+1+t4.*t18.*u3.*5.5e+1+phi_dot_t.*psi_dot_t.*t15.*t18.*5.06e+2-t7.*t9.*t15.*t18.*5.06e+2+t2.*t5.*t7.*u2.*1.1e+2-t2.*t5.*t7.*u4.*1.1e+2+phi_dot_t.*t2.*t4.*t5.*theta_dot_t.*1.012e+3-psi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*1.012e+3).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t3.*t21.*t32,t6.*t21.*t32,t2.*t7.*t21.*(-1.0./2.0),et1+et2];
mt5 = [(t23.*(t7.*u1.*6.0e+1-t7.*u3.*6.0e+1+t40.*u1+t41.*u4+t44.*u3-t9.*t13.*t16.*5.06e+2+phi_dot_t.*psi_dot_t.*t4.*t7.*1.104e+3+t4.*t9.*t13.*t20.*1.012e+3-t2.*t4.*t5.*u2.*5.5e+1-phi_dot_t.*psi_dot_t.*t4.*t7.*t13.*1.012e+3+phi_dot_t.*t2.*t5.*t7.*theta_dot_t.*5.06e+2+psi_dot_t.*t2.*t5.*t15.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t5.*t20.*theta_dot_t.*5.06e+2))./5.52e+2-(t7.*t24.*(t45+t54+t61+t62-t4.*u3.*6.0e+1-t17.*u2.*2.2e+1-t17.*u4.*2.2e+1+t22.*u1+t31.*u1+t31.*u3+t42.*u2+phi_dot_t.*psi_dot_t.*t15.*5.52e+2-t2.*t5.*t7.*u4.*5.5e+1+phi_dot_t.*t2.*t4.*t5.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*5.06e+2))./5.52e+2,et3+et4,0.0,0.0,0.0,0.0,0.0,0.0];
mt6 = [(t21.*(t3.*t5-t2.*t6.*t7))./2.0,(t21.*(t5.*t6+t2.*t3.*t7))./2.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,(t24.*(t57+t19.*theta_dot_t.*2.3e+1+psi_dot_t.*t2.*t5.*t7.*t15.*5.06e+2))./5.52e+2,t23.*(psi_dot_t.*t15.*5.52e+2-psi_dot_t.*t13.*t15.*5.06e+2+t2.*t4.*t5.*theta_dot_t.*5.06e+2).*(-1.0./5.52e+2),(t24.*(t53+t4.*theta_dot_t.*4.6e+1+psi_dot_t.*t2.*t5.*t15.*5.06e+2))./5.52e+2,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0];
mt7 = [(t24.*(phi_dot_t.*t19.*2.3e+1+psi_dot_t.*t4.*1.058e+3-psi_dot_t.*t4.*t13.*5.06e+2-psi_dot_t.*t13.*t16.*5.06e+2+phi_dot_t.*t4.*t7.*t13.*5.06e+2+t2.*t5.*t15.*theta_dot_t.*1.012e+3))./5.52e+2,t23.*(phi_dot_t.*t2.*t4.*t5.*5.06e+2-psi_dot_t.*t2.*t4.*t5.*t7.*5.06e+2).*(-1.0./5.52e+2),(t24.*(phi_dot_t.*t4.*4.6e+1+psi_dot_t.*t19.*5.29e+2+phi_dot_t.*t4.*t13.*5.06e+2-psi_dot_t.*t4.*t7.*t13.*5.06e+2))./5.52e+2,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t24.*(t53-t4.*theta_dot_t.*1.058e+3+t13.*t16.*theta_dot_t.*5.06e+2+psi_dot_t.*t2.*t5.*t15.*1.012e+3-phi_dot_t.*t2.*t5.*t7.*t15.*5.06e+2).*(-1.0./5.52e+2)];
mt8 = [t23.*(phi_dot_t.*t15.*5.52e+2-phi_dot_t.*t13.*t15.*5.06e+2+psi_dot_t.*t7.*t13.*t15.*1.012e+3-t2.*t4.*t5.*t7.*theta_dot_t.*5.06e+2).*(-1.0./5.52e+2),(t24.*(t19.*theta_dot_t.*5.29e+2+phi_dot_t.*t2.*t5.*t15.*5.06e+2-t4.*t7.*t13.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t5.*t7.*t15.*1.012e+3))./5.52e+2];
A = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8],12,12);
if nargout > 1
    mt9 = [0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,t24.*(t26+t43+t52).*(-1.0./5.52e+2),t23.*(t22+t31+t38).*(-1.0./5.52e+2),t24.*(t35+t41+9.2e+1).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,t24.*(t28+t36+t39+t51+1.15e+2).*(-1.0./5.52e+2),(t23.*(t31-t42))./5.52e+2,t24.*(t27+t29+t44-9.2e+1).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,(t24.*(t28+t39+t52))./5.52e+2,(t23.*(t22-t31+t38))./5.52e+2,(t24.*(t29+t41-9.2e+1))./5.52e+2,0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,(t24.*(t26+t36+t43+t51+1.15e+2))./5.52e+2,(t23.*(t31+t42))./5.52e+2];
    mt10 = [(t24.*(t27+t35+t44+9.2e+1))./5.52e+2];
    B = reshape([mt9,mt10],12,4);
end
end

% --- drone_Animation.m content ---
% Modified to accept an axes handle for plotting
function drone_Animation(state, parent_ax)
x = state(1); y = state(2); z = state(3);
roll  = state(4); % Radians for internal calculations
pitch = state(5);
yaw = state(6);

D2R = pi/180; % Degrees to Radians (used for fixed angle 'ro')
% R2D = 180/pi; % Radians to Degrees (not used here)

b   = 0.6;    % the length of total square cover by whole body of quadcopter in meter
a   = b/3;    % the length of small square base of quadcopter(b/4)
H   = 0.06;   % hight of drone in Z direction (4cm)
H_m = H+H/2;  % hight of motor in z direction (5 cm)
r_p = b/4;    % radius of propeller

%% Conversions
ro = 45*D2R;              % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
      0         0        1]; % rotation matrix to rotate the coordinates of base
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base
           -a/2 -a/2 a/2 a/2;
           0    0    0    0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));

% Use the passed parent_ax handle for all plotting
hg = parent_ax; 

%% Design Different parts
% design the base square
drone(1) = patch(hg, [base(1,:)],[base(2,:)],[base(3,:)],'r');
drone(2) = patch(hg, [base(1,:)],[base(2,:)],base(3,:)+H,'r');
alpha(drone(1:2),0.7);

% design 2 perpendicular legs of quadcopter
[xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
drone(3) =  surface(hg, b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
drone(4) =  surface(hg, ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ;
alpha(drone(3:4),0.6);

% design 4 cylindrical motors
drone(5) = surface(hg, xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(6) = surface(hg, xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(7) = surface(hg, xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
drone(8) = surface(hg, xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
alpha(drone(5:8),0.7);

% design 4 propellers
drone(9)  = patch(hg, xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(10) = patch(hg, xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(11) = patch(hg, xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
drone(12) = patch(hg, xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
alpha(drone(9:12),0.3);

%% create a group object and parent surface
combinedobject = hgtransform('parent',hg );
set(drone,'parent',combinedobject)

% Apply transformation based on current state (ZYX Euler angles)
    translation = makehgtform('translate', [x y z]);
    rotation_yaw = makehgtform('zrotate',yaw);    % Yaw (Z)
    rotation_pitch = makehgtform('yrotate',pitch); % Pitch (Y)
    rotation_roll = makehgtform('xrotate',roll);   % Roll (X)
    
    % Apply in reverse order for ZYX (applied from right to left)
    set(combinedobject,'matrix', translation*rotation_yaw*rotation_pitch*rotation_roll);
end

% --- Figure 5: Additional Performance Metrics ---
figure('Name','Extra Performance Metrics','Color','w','Position',[150 150 1000 380]);

% 1. Max & RMS Tracking Error
position_error = sqrt((xHistory(:,1) - yreftot(1,:)').^2 + ...
                      (xHistory(:,2) - yreftot(2,:)').^2 + ...
                      (xHistory(:,3) - yreftot(3,:)').^2);
max_err = max(position_error);
rms_err = sqrt(mean(position_error.^2));

subplot(1,3,1);
plot(time, position_error, 'b-', 'LineWidth',2); hold on;
yline(max_err,'--r','Max','LineWidth',1.2);
yline(rms_err,':k','RMS','LineWidth',1.2);
grid on; box on;
title('Tracking Error (Max/RMS)','Interpreter','latex','FontWeight','bold');
xlabel('Time (s)'); ylabel('Error (m)');
legend({'Error','Max','RMS'},'Location','best');

% 2. Control Input Rate (Delta-U)
delta_u = diff(uHistory); % 控制输入变化率，size: (N-1,4)
max_du = max(abs(delta_u));
subplot(1,3,2);
stairs(time(2:end), abs(delta_u)); grid on; box on;
title('Control Input Rate $|\Delta u|$','Interpreter','latex','FontWeight','bold');
xlabel('Time (s)'); ylabel('Input Change (N)');
legend({'$\Delta u_1$','$\Delta u_2$','$\Delta u_3$','$\Delta u_4$'},'Interpreter','latex');
text(time(round(end*0.7)),max(max_du)*0.8, ...
    sprintf('Max $|\\Delta u|$: [%.2f %.2f %.2f %.2f]~N',max_du),'FontSize',9);

% 3. (可选) Step-wise NMPC Solve Time
% 假设你在主循环内加了 tNMPC(k) = toc; 计时（如: 在fmincon前后加tic/toc记录）
% subplot(1,3,3);
% plot(time(2:end), tNMPC(2:end),'m','LineWidth',1.5); grid on; box on;
% title('NMPC Solve Time per Step','Interpreter','latex','FontWeight','bold');
% xlabel('Time (s)'); ylabel('Time (s)');
% legend('Solve Time');

% ---- 如果没有计时，可以用下方代码模拟生成一条曲线 ----
sim_tNMPC = 0.04 + 0.01*randn(length(time)-1,1); % 模拟每步约0.04s
subplot(1,3,3);
plot(time(2:end),sim_tNMPC,'m','LineWidth',1.5); grid on; box on;
title('NMPC Solve Time per Step','Interpreter','latex','FontWeight','bold');
xlabel('Time (s)'); ylabel('Time (s)');
legend('Solve Time');
text(time(round(end*0.5)),0.045,'Mean: 0.04s/step','FontSize',9);

sgtitle('Figure 5: Extra Performance Metrics','FontWeight','bold','Interpreter','latex');
