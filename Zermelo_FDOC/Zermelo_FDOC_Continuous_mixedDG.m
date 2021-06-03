function phaseout = Zermelo_FDOC_Continuous_mixedDG(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
x1    = input.phase.state(:,1);     % Downstream
x2    = input.phase.state(:,2);     % Upstream
Svec  = input.phase.state(:,3:6);
P1vec = input.phase.state(:,7:10);
P2vec = input.phase.state(:,11:14);

% Extract number of time steps
N = numel(x1);

% -----------------------------------------------------%
% ------ Extract Each Component of the Control ------- %
% -----------------------------------------------------%

u = input.phase.control(:,1);

% ---------------------------------------------------%
% ----------------- Auxiliary data ------------------%
% ---------------------------------------------------%

gamma   = input.auxdata.gamma;
beta    = input.auxdata.beta;

p1      = input.auxdata.p1;
p2      = input.auxdata.p2;
p       = [p1; p2];

Cinf    = input.auxdata.Cinf;
Dinf    = input.auxdata.Dinf;
C2      = input.auxdata.C2;
D2      = input.auxdata.D2;

Q2      = input.auxdata.Q2;
R2      = input.auxdata.R2;
Qinf    = input.auxdata.Qinf;
Rinf    = input.auxdata.Rinf;

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
P1dot = zeros(N, 4);
P2dot = zeros(N, 4);

% Initialize sensitivity matrix dynamics
Sdot = zeros(N, 4);

% Initialize feedback cost
J_feedback = zeros(N,1);

% Loop through each time step 
for k = 1 : N
    
    % Get Riccati vector at time tk
    P1vec_k = P1vec(k,:);
    P2vec_k = P2vec(k,:);
    
    % Get sensitivity vector at time tk
    Svec_k = Svec(k,:);
    
    % Reshape Riccati vector -> Riccati matrix
    P1_k = reshape(P1vec_k, 2, 2);
    P2_k = reshape(P2vec_k, 2, 2);
    
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
    
    % Compute minimizing control and maximizing disturbance
    K_k = -B_k' * P2_k;
    L_k = -(1 / gamma^2) * D_k' * P1_k;
    
    % Propagate sensitivity dynamics
    Sdot_k = (A_k + B_k * K_k) * S_k + D_k;
    
    % Propagate Riccati dynamics
    P1dot_k = -(A_k' * P1_k + P1_k * A_k  - Cinf' * Cinf + ...
                - [P1_k P2_k] * [gamma^(-2) * (D_k * D_k') B_k * B_k'; B_k * B_k' B_k * B_k'] * [P1_k; P2_k]);
    P2dot_k = -(A_k' * P2_k + P2_k * A_k + Cinf' * Cinf + ...
                - [P1_k P2_k] * [zeros(2) gamma^(-2) * (D_k * D_k'); gamma^(-2) * (D_k * D_k') B_k * B_k'] * [P1_k; P2_k]);
    
    % Reshape Riccati matrix -> Riccati vector
    P1dotvec_k = reshape(P1dot_k, 1, 4);
    P2dotvec_k = reshape(P2dot_k, 1, 4);
    
    % Reshape sensitivity matrix -> sensitivity vector
    Sdotvec_k = reshape(Sdot_k, 1, 4);
    
    % Store derivatives
    Sdot(k, :)  = Sdotvec_k;
    P1dot(k, :) = P1dotvec_k;
    P2dot(k, :) = P2dotvec_k; 
    
    % Compute running cost 
    %J_feedback(k) = trace(S_k * SigmaP * S_k') + trace(K_k * S_k * SigmaP * S_k' * K_k') + ...
    %                - gamma^2 * trace(L_k * S_k * SigmaP * S_k' * L_k');
    J_feedback_Hinf_k = 0.5 * (trace(Qinf * S_k * SigmaP * S_k') + trace(Rinf * K_k * S_k * SigmaP * S_k' * K_k') + ...
                            - gamma^2 * trace(L_k * S_k * SigmaP * S_k' * L_k'));
    J_feedback_H2_k   = 0.5 * trace(Q2 * S_k * SigmaP * S_k' + R2 * K_k * S_k * SigmaP * S_k' * K_k');
    J_feedback(k)     = J_feedback_H2_k + beta * J_feedback_Hinf_k;
    
end

% Output dymamics
phaseout.dynamics  = [x1dot, x2dot, Sdot, P1dot, P2dot];

% Ouput Lagrangian (running cost)
phaseout.integrand = J_feedback;
                  
end

    