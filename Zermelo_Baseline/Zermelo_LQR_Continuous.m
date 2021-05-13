function phaseout = Zermelo_LQR_Continuous(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
x1    = input.phase.state(:,1);   % Downstream
x2    = input.phase.state(:,2);   % Upstream
S1    = input.phase.state(:,3); 
S2    = input.phase.state(:,4);
Pvec  = input.phase.state(:,5:8);

% Extract number of time steps
N = numel(x1);

% -----------------------------------------------------%
% ------ Extract Each Component of the Control ------- %
% -----------------------------------------------------%

u = input.phase.control(:,1);

% ---------------------------------------------------%
% ----------------- Auxiliary data ------------------%
% ---------------------------------------------------%

p = input.auxdata.p;
Q = input.auxdata.Q;
R = input.auxdata.R;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%

x1dot   = cos(u) + p * x2;
x2dot   = sin(u);

% ---------------------------------------------------%
% --------- Evaluate Sensitivity Dynamics ---------- %
% ---------------------------------------------------%

Sdot = OpenLoopSensitivityDynamics(N, S1, S2, x1, x2, u, p);

% ---------------------------------------------------%
% --------- Evaluate Riccati Dynamics -------------- %
% ---------------------------------------------------%

Pdot = RiccatiMatrixDynamics(N, Pvec, Q, R, x1, x2, u, p);


% Output dymamics
phaseout.dynamics  = [x1dot, x2dot, Sdot, Pdot];
                  
end

    