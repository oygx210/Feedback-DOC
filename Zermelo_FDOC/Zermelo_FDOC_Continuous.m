function phaseout = Zermelo_FDOC_Continuous(input)

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

p       = input.auxdata.p;
Q       = input.auxdata.Q;
R       = input.auxdata.R;
SigmaP  = input.auxdata.SigmaP;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%

x1dot   = cos(u) + p * x2;
x2dot   = sin(u);

% -----------------------------------------------------------------%
% ---------------------- Evaluate Feedback Cost ------------------ %
% -----------------------------------------------------------------%

% Initialize Riccati matrix dynamics
Pdot = zeros(N, 4);

% Initialize sensitivity matrix dynamics
Sdot = zeros(N, 2);

% Initialize feedback cost
J_feedback = zeros(N,1);

% Loop through each time step 
for k = 1 : N
    
    % Get Riccati vector at time tk
    Pvec_k = Pvec(k,:);
    
    % Reshape Riccati vector -> Riccati matrix
    P_k = reshape(Pvec_k, 2, 2);
    
    % Get sensitivity matrix
    S_k = [S1(k); S2(k)];
    
    % Get state at time tk
    x1_k = x1(k);
    x2_k = x2(k);
    x_k = [x1_k; x2_k];
    
    % Get control at time tk
    u_k = u(k);
    
    % Compute linearizations at time tk
    A_k = computeStateLinearization(x_k, u_k, p);
    B_k = computeControlLinearization(x_k, u_k, p);
    D_k = computeParameterLinearization(x_k, u_k, p);
    
    % Compute Kalman gain
    K_k = -R \ B_k' * P_k;
    
    % Propagate sensitivity dynamics
    Sdot_k = (A_k + B_k * K_k) * S_k + D_k;
    
    % Propagate Riccati dynamics
    Pdot_k = -(A_k' * P_k + P_k * A_k + Q - P_k * B_k / R * B_k' * P_k);
    
    % Reshape Riccati matrix -> Riccati vector
    Pdotvec_k = reshape(Pdot_k, 1, 4);
    
    % Store derivatives
    Sdot(k, :) = Sdot_k';
    Pdot(k, :) = Pdotvec_k;
    
    % Store running cost
    J_feedback(k) = 0.5 * trace(Q * S_k * SigmaP * S_k' + R * K_k * S_k * SigmaP * S_k' * K_k');

end

% Output dymamics
phaseout.dynamics  = [x1dot, x2dot, Sdot, Pdot];

% Ouput Lagrangian (running cost)
phaseout.integrand = J_feedback;
                  
end

    