%% Housekeeping
clear all; close all; clc;
addpath('Linearizations','OtherFunctions','Plotting','Zermelo_Baseline','Zermelo_FDOC','Zermelo_DOC');

%% Problem setup
[C, IC, FC, LB, UB] = setup_FDOC_mixedDG();

%% Solve nominal OCP
nominal = obtain_baseline_solution_v2(C, IC, FC, LB, UB);

%% Plot nominal state and control
postProcess_v2(nominal,'k');

%% Solve open-loop DOCP
openloop = obtain_DOC_solution(C, IC, FC, LB, UB);

%% Plot desensitized state and control (OL)
postProcess_DOC(openloop,'k');

%% Solve LQG feedback DOCP
feedbackLQR  = obtain_FDOC_solution_v2(C, IC, FC, LB, UB);

%% Plot desensitized state and control (LQG feedback)
fdoc_LQR_plot_style = {'--k','LineWidth',1.5};
postProcess_FDOC_v2(feedbackLQR,fdoc_LQR_plot_style);

%% Solve mixed H2 / Hinf feedback DOCP
feedbackMixed = obtain_FDOC_solution_mixedDG(C, IC, FC, LB, UB);

%% Plot desensitized state and control (Hinf feedback)
fdoc_mixed_plot_style = {'k','LineWidth',1.5};
postProcess_FDOC_mixed(feedbackMixed,fdoc_mixed_plot_style);

%% Monte Carlo Analysis

% Generate 100 samples for parameters
rng(0);
num_MC = 100;
p_MC = mvnrnd(C.p, C.SigmaP, num_MC);

% Plotting styles and covariance ellipsoids

bl_scatter_style         = {'o', 'MarkerEdgeColor', 0.4 * ones(3,1)};
doc_scatter_style        = {'.', 'MarkerEdgeColor', 0.2 * ones(3,1),'LineWidth',2.5};
fdoc_mixed_scatter_style = {'k+','LineWidth',2};
fdoc_LQR_scatter_style   = {'kx','LineWidth',1};
bl_plot_style            = {'-.', 'Color', 0.25 * ones(3, 1)};
doc_plot_style           = {':', 'Color', 0.1 * ones(3, 1),'LineWidth',1.5};
fdoc_LQR_plot_style      = {'--k','LineWidth',1.5};
fdoc_mixed_plot_style    = {'k','LineWidth',1.5};
conf                     = 0.9973;
Npl                      = 100;

% Plot samples
figure; hold on; grid on; box on;
xlabel('Current Strength $x$','Interpreter','latex','FontSize',15);
ylabel('Current Strength $y$','Interpreter','latex','FontSize',15);
set(gca,'FontSize',15);
scatter(p_MC(:,1), p_MC(:,2), fdoc_mixed_scatter_style{:});
conf_ellipse(C.p,C.SigmaP,Npl,conf,1,'--k');

% Get optimal LQR CL desensitized control and times
t_FDOC_LQR   = feedbackLQR.phase.time;
u_FDOC_LQR   = feedbackLQR.phase.control;
P_LQR        = feedbackLQR.phase.state(7:10);

% Get optimal Hinf CL desensitized control and times
t_FDOC_mixed  = feedbackMixed.phase.time;
u_FDOC_mixed  = feedbackMixed.phase.control;

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
xhist_FDOC_mixed = zeros(M, 2, num_MC);
xhist_FDOC_LQR   = zeros(M, 2, num_MC);
xhist_DOC        = zeros(M, 2, num_MC);
xhist_BL         = zeros(M, 2, num_MC);

% ODE45 options
options = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Loop through each parameter value
for ii = 1 : num_MC
    
    % Get MC parameter value
    p_ii = p_MC(ii, :);
    
    % Propagate dynamics 
    [tout_FDOC_mixed, xout_FDOC_mixed] = ode45(@(t,x) ZermeloFeedbackDynamics_v2(t,x,feedbackMixed,C,p_ii,2), tspan, x0, options); 
    [tout_FDOC_LQR, xout_FDOC_LQR]     = ode45(@(t,x) ZermeloFeedbackDynamics_v2(t,x,feedbackLQR,C,p_ii,1), tspan, x0, options);
    [tout_DOC, xout_DOC]               = ode45(@(t,x) ZermeloDynamics_v2(t,x,t_DOC,u_DOC,p_ii), tspan, x0, options);
    [tout_BL, xout_BL]                 = ode45(@(t,x) ZermeloDynamics_v2(t,x,t_BL,u_BL,p_ii), tspan, x0, options);

    % Store MC trajectory
    xhist_FDOC_mixed(:, :, ii) = xout_FDOC_mixed;
    xhist_FDOC_LQR(:, :, ii)  = xout_FDOC_LQR;
    xhist_DOC(:, :, ii)       = xout_DOC;
    xhist_BL(:, :, ii)        = xout_BL;
    
end

% Interpolate nominal trajectories
x_FDOC_mixed          = feedbackMixed.phase.state;
x_FDOC_LQR            = feedbackLQR.phase.state;
x_DOC                 = openloop.phase.state;
x_BL                  = nominal.phase.state;
x_FDOC_mixed_nominal  = zeros(M, 2);
x_FDOC_LQR_nominal    = zeros(M, 2);
x_DOC_nominal         = zeros(M, 2);
x_BL_nominal          = zeros(M, 2);

for k = 1 : M
    
    tk = tspan(k);
    
    full_state_FDOC_mixed_k    = interp1(t_FDOC_mixed, x_FDOC_mixed, tk);
    x_FDOC_mixed_nominal(k, :) = full_state_FDOC_mixed_k(1:2);
    
    full_state_FDOC_LQR_k      = interp1(t_FDOC_LQR, x_FDOC_LQR, tk);
    x_FDOC_LQR_nominal(k, :)   = full_state_FDOC_LQR_k(1:2);
    
    full_state_DOC_k           = interp1(t_DOC, x_DOC, tk);
    x_DOC_nominal(k, :)        = full_state_DOC_k(1:2);
    
    full_state_BL_k            = interp1(t_BL, x_BL, tk);
    x_BL_nominal(k, :)         = full_state_BL_k(1:2);
    
end

% Plot nominal and MC trajectories
grey = [0.7 0.7 0.7];
figure; 
subplot(2,2,1); hold on; grid on;
title('Open Loop Baseline','Interpreter','latex','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_BL(:,1,ii),xhist_BL(:,2,ii),'color',grey);
end
plot(x_BL_nominal(:,1),x_BL_nominal(:,2),'k','LineWidth',2);
subplot(2,2,2); hold on; grid on;
title('Open Loop Desensitized','Interpreter','latex','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_DOC(:,1,ii),xhist_DOC(:,2,ii),'color',grey);
end
plot(x_DOC_nominal(:,1),x_DOC_nominal(:,2),'k','LineWidth',2);
subplot(2,2,3); hold on; grid on;
title('Closed Loop LQR Feedback Desensitized','Interpreter','latex','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_FDOC_LQR(:,1,ii),xhist_FDOC_LQR(:,2,ii),'color',grey);
end
plot(x_FDOC_LQR_nominal(:,1),x_FDOC_LQR_nominal(:,2),'k','LineWidth',2);
subplot(2,2,4); hold on; grid on;
title('Closed Loop $H_{\infty}$ Feedback Desensitized','Interpreter','latex','FontSize',12);
xlabel('Downstream','FontSize',15);
ylabel('Upstream','FontSize',15);
set(gca,'FontSize',15);
for ii = 1 : num_MC
    plot(xhist_FDOC_mixed(:,1,ii),xhist_FDOC_mixed(:,2,ii),'color',grey);
end
plot(x_FDOC_mixed_nominal(:,1),x_FDOC_mixed_nominal(:,2),'k','LineWidth',2);

% Compute errors between MC and nominal
error_FDOC_mixed       = zeros(M, 2, num_MC);
error_FDOC_LQR         = zeros(M, 2, num_MC);
error_DOC              = zeros(M, 2, num_MC);
error_BL               = zeros(M, 2, num_MC);
final_error_FDOC_mixed = zeros(num_MC, 2);
final_error_FDOC_LQR   = zeros(num_MC, 2);
final_error_DOC        = zeros(num_MC, 2);
final_error_BL         = zeros(num_MC, 2);

for ii = 1 : num_MC
    
    error_FDOC_mixed(:, :, ii)    = x_FDOC_mixed_nominal - xhist_FDOC_mixed(:, :, ii); 
    error_FDOC_LQR(:, :, ii)     = x_FDOC_LQR_nominal - xhist_FDOC_LQR(:, :, ii);
    error_DOC(:, :, ii)          = x_DOC_nominal - xhist_DOC(:, :, ii);
    error_BL(:, :, ii)           = x_BL_nominal - xhist_BL(:, :, ii);
    
    final_error_FDOC_mixed(ii, :) = error_FDOC_mixed(end, :, ii);
    final_error_FDOC_LQR(ii, :)  = error_FDOC_LQR(end, :, ii);
    final_error_DOC(ii, :)       = error_DOC(end, :, ii);
    final_error_BL(ii, :)        = error_BL(end, :, ii);
    
end

% Compute covariances and means
covariance_FDOC_mixed = cov(final_error_FDOC_mixed);
covariance_FDOC_LQR   = cov(final_error_FDOC_LQR);
covariance_DOC        = cov(final_error_DOC);
covariance_BL         = cov(final_error_BL);
mean_FDOC_Hinf        = mean(final_error_FDOC_mixed)';
mean_FDOC_LQR         = mean(final_error_FDOC_LQR)';
mean_DOC              = mean(final_error_DOC)';
mean_BL               = mean(final_error_BL)';

% Ellipse semi major axes
length_BL         = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_BL)));
length_DOC        = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_DOC)));
length_FDOC_mixed = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_FDOC_mixed)));
length_FDOC_LQR   = sqrt(-2 * log(1 - conf)) * sqrt(max(eig(covariance_FDOC_LQR)));

% Plot final dispersions for baseline and FDOC
figure; hold on; grid on;
xlabel('Final downstream error','FontSize',15);
ylabel('Final upstream error','FontSize',15);
scatter(final_error_FDOC_mixed(:,1),final_error_FDOC_mixed(:,2),fdoc_mixed_scatter_style{:});
scatter(final_error_FDOC_LQR(:,1),final_error_FDOC_LQR(:,2),fdoc_LQR_scatter_style{:});
scatter(final_error_DOC(:,1),final_error_DOC(:,2),doc_scatter_style{:});
scatter(final_error_BL(:,1),final_error_BL(:,2),bl_scatter_style{:});
conf_ellipse(mean_FDOC_Hinf,covariance_FDOC_mixed,Npl,conf,1,fdoc_mixed_plot_style{:});
conf_ellipse(mean_FDOC_LQR,covariance_FDOC_LQR,Npl,conf,1,fdoc_LQR_plot_style{:});
conf_ellipse(mean_DOC,covariance_DOC,Npl,conf,1,doc_plot_style{:});
conf_ellipse(mean_BL,covariance_BL,Npl,conf,1,bl_plot_style{:});
set(gca,'FontSize',15);
legend({'$H_{\infty}$ Feedback','LQR Feedback','Open Loop','Baseline'},...
       'Interpreter','latex','FontSize',15,'Location','Best');

    
    
    
    
    
