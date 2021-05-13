function phaseout = Zermelo_baseline_Continuous_v2(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
x1    = input.phase.state(:,1);     % Downstream
x2    = input.phase.state(:,2);     % Upstream
Svec  = input.phase.state(:,3:6);

% Extract number of time steps
N = numel(x1);

% -----------------------------------------------------%
% ------ Extract Each Component of the Control ------- %
% -----------------------------------------------------%

u = input.phase.control(:,1);

% ---------------------------------------------------%
% ----------------- Auxiliary data ------------------%
% ---------------------------------------------------%

p1 = input.auxdata.p1;
p2 = input.auxdata.p2;
p = [p1; p2];

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%

x1dot   = cos(u) + p1 * x2;
x2dot   = sin(u) + p2;

% ---------------------------------------------------%
% --------- Evaluate Sensitivity Dynamics ---------- %
% ---------------------------------------------------%

Sdot = OpenLoopSensitivityDynamics_v2(N, Svec, x1, x2, u, p);


% Output dymamics
phaseout.dynamics  = [x1dot, x2dot, Sdot];
                  
end

    