%-------------------------------------------------------------------%
%--- Set all the constants, limits, initial and final conditions ---%
%-------------------------- appropriately --------------------------%
%-------------------------------------------------------------------%

function [C, IC, FC, LB, UB] = setup_FDOC()

% C  - Constants
% IC - Initial conditions
% FC - Final conditions
% LB - Lower bounds
% UB - Upper bounds

%% Important constants

C.p      = 10;
C.SigmaP = (5/3)^2;
C.Qf     = 2 * eye(2);
C.Q      = eye(2);
C.R      = 10;
C.alpha  = 100;

%% Initial conditions

IC.time  = 0;              % time
IC.x1    = 0;              % downstream 
IC.x2    = 0;              % upstream 
IC.Svec  = zeros(1,2);     % sensitivity matrix   (vectorized)

%% Final conditions

FC.x2    = 0;
FC.Pvec  = reshape(C.Qf,1,4);

%% Limits (or bounds)

% Final time
LB.tf     = 1; 
UB.tf     = 1;

% Downstream
LB.x1     = 0;
UB.x1     = 10;

% Upstream
LB.x2     = 0;
UB.x2     = 5;

% Steering angle
LB.u      = -pi/2;
UB.u      = pi/2;

% Sensitivity Matrix
LB.Svec  = -1E5 * ones(1,2); 
UB.Svec  = 1E5 * ones(1,2);

% Riccati Matrix
LB.Pvec  = -1E5 * ones(1,4);
UB.Pvec  = 1E5 * ones(1,4);

end