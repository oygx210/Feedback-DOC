% ------------- Hypersonic vehicle Trajectory Optimization -------------- %
% ------------------ Author: Venkata Ramana Makkapati ------------------- %
% -- Run final time desensitization while penalizing NS, EW ranges and ---% 
% ------------------------- energy derivatives  ------------------------- %
% --- (uncertain parameters: Drag parameter - CD0; Scale height - H) -- - %

close all;
clear all;
clc;

% Choose alpha values for your desensitization 
% 0 implies no desensitization
alpha = 0;

% Load the set-up (constants, initial and final conditions, bounds)
[C, IC, FC, LB, UB] = setup();

% Obtain some important conversions
run conversions

% Nondimensionalization
run ND_processing

% Sample 100 parameters values within +-2% of nominal one for 
% Monte Carlo simulations
C.SigmaP = (0.02 / 3*[C.Hbar, 0;0, C.CD0]).^2;
p_nom    = [C.Hbar; C.CD0];
n_trials = 1000;

% Normal distribution
p_range = mvnrnd(p_nom, C.SigmaP, n_trials)';
    
% Obtain solution from GPOPS-II
solution = obtain_solution(alpha, C, IC, FC, LB, UB, ND);

% for i = 1:numel(alpha_values)
% 
%     % Run open-loop MC simulation
%     open_MC(i) = perform_open_MC(solution(i), C, IC, p_range, i, ND, alpha_values);
% end
% 
% jack_plots
