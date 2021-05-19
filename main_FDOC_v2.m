%% Housekeeping
clear all; close all; clc;
addpath('Linearizations','OtherFunctions','Plotting','Zermelo_Baseline','Zermelo_FDOC','Zermelo_DOC');

%% Problem setup
[C, IC, FC, LB, UB] = setup_FDOC_v2();

%% Solve nominal OCP
nominal = obtain_baseline_solution_v2(C, IC, FC, LB, UB);

%% Plot nominal state and control
postProcess_v2(nominal,'k');

%% Solve open-loop DOCP
openloop = obtain_DOC_solution(C, IC, FC, LB, UB);

%% Plot desensitized state and control (OL)
postProcess_DOC(openloop,'k');

%% Solve feedback DOCP
feedback = obtain_FDOC_solution_v2(C, IC, FC, LB, UB);

%% Plot desensitized state and control (feedback)
postProcess_FDOC_v2(feedback,'k');

%% Solve gain-scheduled Hinf Problem
%Kt = obtain_Hinf_solution(feedback)

%% Monte Carlo Analysis

% Generate 100 samples for parameters
rng(0);
num_MC = 100;
p_MC = mvnrnd(C.p, C.SigmaP, num_MC);

% Plotting styles and covariance ellipsoids

bl_scatter_style   = {'o', 'MarkerEdgeColor', 0.4 * ones(3,1)};
doc_scatter_style  = {'.', 'MarkerEdgeColor', 0.2 * ones(3,1),'LineWidth',1.5};
fdoc_scatter_style = {'k+','LineWidth',2};
bl_plot_style      = {'-.', 'Color', 0.25 * ones(3, 1)};
doc_plot_style     = {':', 'Color', 0.1 * ones(3, 1),'LineWidth',1.5};
fdoc_plot_style    = {'k'};
conf            = 0.9973;
Npl             = 100;

% Plot samples
figure; hold on; grid on; box on;
xlabel('Current Strength','FontSize',15);
set(gca,'FontSize',15);
scatter(p_MC(:,1), p_MC(:,2), fdoc_scatter_style{:});
conf_ellipse(C.p,C.SigmaP,Npl,conf,1,'--k');

% Get optimal CL desensitized control and times
t_FDOC   = feedback.phase.time;
u_FDOC   = feedback.phase.control;
P        = feedback.phase.state(7:10);

% Get optimal OL desensitized control and times
t_DOC    = openloop.phase.time;
u_DOC    = openloop.phase.control;

% Get baseline control and times
t_BL     = nominal.phase.time;
u_BL     = nominal.phase.control;

% Construct time vector
M = 100;
tspan = linspace(0, 1, M);

% Initial state
x0 = zeros(2,1);

% Initialize MC state histories
xhist_FDOC = zeros(M, 2, num_MC);
xhist_DOC  = zeros(M, 2, num_MC);
xhist_BL   = zeros(M, 2, num_MC);

% ODE45 options
options = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Loop through each parameter value
for ii = 1 : num_MC
    
    % Get MC parameter value
    p_ii = p_MC(ii, :);
    
    % Propagate dynamics 
    [tout_FDOC, xout_FDOC]   = ode45(@(t,x) ZermeloFeedbackDynamics_v2(t,x,feedback,C,p_ii), tspan, x0, options);
    [tout_DOC, xout_DOC]     = ode45(@(t,x) ZermeloDynamics_v2(t,x,t_DOC,u_DOC,p_ii), tspan, x0, options);
    [tout_BL, xout_BL]       = ode45(@(t,x) ZermeloDynamics_v2(t,x,t_BL,u_BL,p_ii), tspan, x0, options);

    % Store MC trajectory
    xhist_FDOC(:, :, ii) = xout_FDOC;
    xhist_DOC(:, :, ii)  = xout_DOC;
    xhist_BL(:, :, ii)   = xout_BL;
    
end

% Interpolate nominal trajectories
x_FDOC          = feedback.phase.state;
x_DOC           = openloop.phase.state;
x_BL            = nominal.phase.state;
x_FDOC_nominal  = zeros(M, 2);
x_DOC_nominal   = zeros(M, 2);
x_BL_nominal    = zeros(M, 2);

for k = 1 : M
    
    tk = tspan(k);
    
    full_state_FDOC_k    = interp1(t_FDOC, x_FDOC, tk);
    x_FDOC_nominal(k, :) = full_state_FDOC_k(1:2);
    
    full_state_DOC_k     = interp1(t_DOC, x_DOC, tk);
    x_DOC_nominal(k, :)  = full_state_DOC_k(1:2);
    
    full_state_BL_k      = interp1(t_BL, x_BL, tk);
    x_BL_nominal(k, :)   = full_state_BL_k(1:2);
    
end

% Plot nominal and MC trajectories
grey = [0.7 0.7 0.7];
figure; 
subplot(1,3,1); hold on; grid on;
title('Open Loop Baseline','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_BL(:,1,ii),xhist_BL(:,2,ii),'color',grey);
end
plot(x_BL_nominal(:,1),x_BL_nominal(:,2),'k','LineWidth',2);
subplot(1,3,2); hold on; grid on;
title('Open Loop Desensitized','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_DOC(:,1,ii),xhist_DOC(:,2,ii),'color',grey);
end
plot(x_DOC_nominal(:,1),x_DOC_nominal(:,2),'k','LineWidth',2);
subplot(1,3,3); hold on; grid on;
title('Closed Loop Feedback Desensitized','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_FDOC(:,1,ii),xhist_FDOC(:,2,ii),'color',grey);
end
plot(x_FDOC_nominal(:,1),x_FDOC_nominal(:,2),'k','LineWidth',2);

% Compute errors between MC and nominal
error_FDOC        = zeros(M, 2, num_MC);
error_DOC         = zeros(M, 2, num_MC);
error_BL          = zeros(M, 2, num_MC);
final_error_FDOC  = zeros(num_MC, 2);
final_error_DOC   = zeros(num_MC, 2);
final_error_BL    = zeros(num_MC, 2);

for ii = 1 : num_MC
    
    error_FDOC(:, :, ii)     = x_FDOC_nominal - xhist_FDOC(:, :, ii);
    error_DOC(:, :, ii)      = x_DOC_nominal - xhist_DOC(:, :, ii);
    error_BL(:, :, ii)       = x_BL_nominal - xhist_BL(:, :, ii);
    
    final_error_FDOC(ii, :)  = error_FDOC(end, :, ii);
    final_error_DOC(ii, :)   = error_DOC(end, :, ii);
    final_error_BL(ii, :)    = error_BL(end, :, ii);
    
end

% Compute covariances and means
covariance_FDOC = cov(final_error_FDOC);
covariance_DOC  = cov(final_error_DOC);
covariance_BL   = cov(final_error_BL);
mean_FDOC       = mean(final_error_FDOC)';
mean_DOC        = mean(final_error_DOC)';
mean_BL         = mean(final_error_BL)';

% Ellipse semi major axes
length_BL    = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_BL)));
length_DOC   = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_DOC)));
length_FDOC  = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_FDOC)));

% Plot final dispersions for baseline and FDOC
figure; hold on; grid on;
xlabel('Final downstream error','FontSize',15);
ylabel('Final upstream error','FontSize',15);
scatter(final_error_FDOC(:,1),final_error_FDOC(:,2),fdoc_scatter_style{:});
scatter(final_error_DOC(:,1),final_error_DOC(:,2),doc_scatter_style{:});
scatter(final_error_BL(:,1),final_error_BL(:,2),bl_scatter_style{:});
conf_ellipse(mean_FDOC,covariance_FDOC,Npl,conf,1,fdoc_plot_style{:});
conf_ellipse(mean_DOC,covariance_DOC,Npl,conf,1,doc_plot_style{:});
conf_ellipse(mean_BL,covariance_BL,Npl,conf,1,bl_plot_style{:});
set(gca,'FontSize',15);
legend({'Feedback Desensitized','Open Loop Desensitized','Baseline'},'FontSize',15,'Location','Best');

    
    
    
    
    
