%-------------------------------------------------------------------%
%--- Set all the constants, limits, initial and final conditions ---%
%-------------------------- appropriately --------------------------%
%-------------------------------------------------------------------%

function [C, IC, FC, LB, UB] = setup()

% C  - Constants
% IC - Initial conditions
% FC - Final conditions
% LB - Lower bounds
% UB - Upper bounds

%% Important constants

C.Re   = 6371000;           % Equatorial Radius of Earth (m)
C.S    = 150;               % Vehicle Reference Area (m^2)
C.H    = 7254.24;           % Density Scale Height (m)
C.rho0 = 1.225;             % Sea Level Atmospheric Density (kg/m^3)
C.g0   = 9.81;              % Gravity at sea level (m/s^2) 
C.mass = 38000;             % Vehicle Mass (kg)
C.kQ   = 9.4369 * 1e-5;     % heating rate constant

C.Qf   = blkdiag(0, 1, 1, 0, 0, 0); % terminal output penalty
C.R    = 0.1 * eye(2);

% Parameters for Drag Coefficient
% X-33 drag-polar: CD = CD0 + K * CL ^ N
C.CD0 = 0.12; 
C.K   = 1.125; 
C.N   = 1.9;

C.Qdotmax = 400000;   % Maximum heating rate allowed
C.qbarmax = 14500;    % Maximum dynamic pressure allowed
C.nmax    = 5 * C.g0; % Maximum normal load allowed/gravity

% Obtain Gravitational constant (m^3 / s^2)
C.mu   = C.g0 * C.Re^2;

%% Initial conditions

IC.time    = 0;              % time (s)
IC.alt     = 121900;         % altitude (m) 121900
IC.lon     = -123*pi/180;    % longitude (rad)
IC.lat     = -25*pi/180;     % latitude (rad)
IC.speed   = 7626;           % speed (m/s)
IC.fpa     = -1.2493*pi/180; % flight path angle (rad)
IC.azi     = +45*pi/180;     % azimuth (rad)
IC.rad     = IC.alt + C.Re; 
IC.Svec    = zeros(1, 12);

%% Final conditions

FC.time    = 1200;            % time (s)
FC.alt     = 30480;           % altitude (m)
FC.lon     = -81*pi/180;      % longitude (rad)
FC.lat     = +28.61*pi/180;   % latitude (rad)
FC.speed   = 908.15;          % speed (m/s)
FC.fpaMin  = -6*pi/180;       % minimum flight path angle (rad)
FC.fpaMax  = 0*pi/180;        % maximum flight path angle (rad)
FC.azi     = +90*pi/180;      % azimuth (rad)
FC.rad     = FC.alt + C.Re; 
FC.Pvec    = reshape(C.Qf, 1, 36);

%% Limits (or bounds)

% % Final time (s)
% LB.tf  = 0; 
% UB.tf  = 3000;

% radius (m)
LB.rad = C.Re; 
UB.rad = IC.rad;

% Longitude (rad)
LB.lon = -pi;         
UB.lon = -LB.lon;

% Latitude (rad)
LB.lat = -70 * pi / 180;  
UB.lat = -LB.lat;

% Speed (m/s)
LB.speed = 10;        
UB.speed = 45000;

% Flight path angle (rad)
LB.fpa = -80 * pi / 180;  
UB.fpa = -LB.fpa;

% Azimuth
LB.azi = -180 * pi / 180; 
UB.azi = -LB.azi;

% Lift coefficient
LB.CL  = -0.15;       
UB.CL  = 0.8; 

% Bank angle (rad)
LB.bank  = -90 * pi/180;
UB.bank  = 90 * pi/180;

% Control derivatives
LB.CLdot   = -0.05; 
UB.CLdot   = -LB.CLdot;

LB.bankdot = -5 * (pi / 180); 
UB.bankdot = -LB.bankdot;

% Sensitivity Matrix
LB.Svec  = -1E5 * ones(1, 12); 
UB.Svec  = 1E5 * ones(1, 12);

% Riccati Matrix
LB.Pvec  = -1E5 * ones(1, 36);
UB.Pvec  = 1E5 * ones(1, 36);

end