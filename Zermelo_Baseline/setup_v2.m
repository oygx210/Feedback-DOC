%-------------------------------------------------------------------%
%--- Set all the constants, limits, initial and final conditions ---%
%-------------------------- appropriately --------------------------%
%-------------------------------------------------------------------%

function [C, IC, FC, LB, UB] = setup_v2()

% C  - Constants
% IC - Initial conditions
% FC - Final conditions
% LB - Lower bounds
% UB - Upper bounds

%% Important constants

C.p1      = 10;
C.p2      = 0;
C.p       = [C.p1; C.p2];

%% Initial conditions

IC.time  = 0;              % time
IC.x1    = 0;              % downstream 
IC.x2    = 0;              % upstream 
IC.Svec  = zeros(1,4);     % sensitivity matrix (vectorized)

%% Final conditions

FC.x2    = 0;

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

% Sensitivities
LB.Svec  = -1E5 * ones(1,4); 
UB.Svec  = 1E5 * ones(1,4);

end