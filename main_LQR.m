%% Housekeeping
clear all; close all; clc;
addpath('Linearizations','OtherFunctions','Plotting','Zermelo_Baseline','Zermelo_FDOC');

%% Problem setup
[C, IC, FC, LB, UB] = setup_plus_LQR();

%% Solve nominal OCP
nominal = obtain_baseline_solution_plus_LQR(C, IC, FC, LB, UB);

%% Plot nominal state and control
postProcess_LQR(nominal);
