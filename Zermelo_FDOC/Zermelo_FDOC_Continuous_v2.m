function phaseout = Zermelo_FDOC_Continuous_v2(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
x1    = input.phase.state(:,1);     % Downstream
x2    = input.phase.state(:,2);     % Upstream
Svec  = input.phase.state(:,3:6);
Pvec  = input.phase.state(:,7:10);

% Extract number of time steps
N = numel(x1);

% -----------------------------------------------------%
% ------ Extract Each Component of the Control ------- %
% -----------------------------------------------------%

u = input.phase.control(:,1);

% ---------------------------------------------------%
% ----------------- Auxiliary data ------------------%
% ---------------------------------------------------%

p1      = input.auxdata.p1;
p2      = input.auxdata.p2;
p       = [p1; p2];
Q       = input.auxdata.Q;
R       = input.auxdata.R;
SigmaP  = input.auxdata.SigmaP;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%

x1dot   = cos(u) + p1 * x2;
x2dot   = sin(u) + p2;

% -----------------------------------------------------------------%
% ---------------------- Evaluate Feedback Cost ------------------ %
% -----------------------------------------------------------------%

% Initialize Riccati matrix dynamics
Pdot = zeros(N, 4);

% Initialize sensitivity matrix dynamics
Sdot = zeros(N, 4);

% Initialize feedback cost
J_feedback = zeros(N,1);

% Loop through each time step 
for k = 1 : N
    
    % Get Riccati vector at time tk
    Pvec_k = Pvec(k,:);
    
    % Get sensitivity vector at time tk
    Svec_k = Svec(k,:);
    
    % Reshape Riccati vector -> Riccati matrix
    P_k = reshape(Pvec_k, 2, 2);
    
    % Reshape sensitivity vector -> sensitivity matrix
    S_k = reshape(Svec_k, 2, 2);
    
    % Get state at time tk
    x1_k = x1(k);
    x2_k = x2(k);
    x_k = [x1_k; x2_k];
    
    % Get control at time tk
    u_k = u(k);
    
    % Compute linearizations at time tk
    A_k = computeStateLinearization_v2(x_k, u_k, p);
    B_k = computeControlLinearization(x_k, u_k, p);
    D_k = computeParameterLinearization_v2(x_k, u_k, p);
    
    % Compute Kalman gain
    K_k = -R \ B_k' * P_k;
    
    % Propagate sensitivity dynamics
    Sdot_k = (A_k + B_k * K_k) * S_k + D_k;
    
    % Propagate Riccati dynamics
    Pdot_k = -(A_k' * P_k + P_k * A_k + Q - P_k * B_k / R * B_k' * P_k);
    
    % Reshape Riccati matrix -> Riccati vector
    Pdotvec_k = reshape(Pdot_k, 1, 4);
    
    % Reshape sensitivity matrix -> sensitivity vector
    Sdotvec_k = reshape(Sdot_k, 1, 4);
    
    % Store derivatives
    Sdot(k, :) = Sdotvec_k;
    Pdot(k, :) = Pdotvec_k;
    
    % Store running cost
    J_feedback(k) = 0.5 * trace(Q * S_k * SigmaP * S_k' + R * K_k * S_k * SigmaP * S_k' * K_k');

end

% Output dymamics
phaseout.dynamics  = [x1dot, x2dot, Sdot, Pdot];

% Ouput Lagrangian (running cost)
phaseout.integrand = J_feedback;
                  
end

    