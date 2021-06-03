function phaseout = Zermelo_FDOC_Continuous_mixed(input)

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

gamma   = input.auxdata.gamma;
p1      = input.auxdata.p1;
p2      = input.auxdata.p2;
p       = [p1; p2];
SigmaP  = input.auxdata.SigmaP;
C2      = input.auxdata.C2;
D2      = input.auxdata.D2;
Cinf    = input.auxdata.Cinf;
Dinf    = input.auxdata.Dinf;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%

x1dot   = cos(u) + p1 * x2;
x2dot   = sin(u) + p2;

% -----------------------------------------------------------------%
% ---------------------- Evaluate Feedback Cost ------------------ %
% -----------------------------------------------------------------%

% Initialize sensitivity matrix dynamics
Sdot = zeros(N, 4);

% Initialize feedback cost
J_feedback = zeros(N,1);

% Loop through each time step 
for k = 1 : N
    
    % Get sensitivity vector at time tk
    Svec_k = Svec(k,:);
    
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
    
    % Compute minimizing feedback control K_k
    
    X0   = zeros(8, 1);
    Xk   = fsolve(@(X) coupledRiccati(X,A_k,B_k,D_k,Cinf,C2,gamma), X0);  
    
    %solution_k = mixed_L2_Linf_LMI(A_k,B_k,D_k,C2,D2,Cinf,Dinf,gamma);
    
    % Propagate sensitivity dynamics
    Sdot_k = (A_k + B_k * K_k) * S_k + D_k;
    %Sdot_k = (A_k + B_k * K_k + D_k * L_k) * S_k;
    
    % Propagate Riccati dynamics
    X_k    = B_k / R * B_k' - (1/ gamma^2) * D_k / Rp * D_k';
    Pdot_k = -(A_k' * P_k + P_k * A_k  - P_k * X_k * P_k + Q);
    
    % Reshape Riccati matrix -> Riccati vector
    Pdotvec_k = reshape(Pdot_k, 1, 4);
    
    % Reshape sensitivity matrix -> sensitivity vector
    Sdotvec_k = reshape(Sdot_k, 1, 4);
    
    % Store derivatives
    Sdot(k, :) = Sdotvec_k;
    Pdot(k, :) = Pdotvec_k;
    
    % Compute running cost 
    J_feedback(k) = 0.5 * (trace(Q * S_k * SigmaP * S_k') + trace(R * K_k * S_k * SigmaP * S_k' * K_k') + ...
                    - gamma^2 * trace(Rp * L_k * S_k * SigmaP * S_k' * L_k'));

end

% Output dymamics
phaseout.dynamics  = [x1dot, x2dot, Sdot, Pdot];

% Ouput Lagrangian (running cost)
phaseout.integrand = J_feedback;
                  
end

function out = coupledRiccati(X,A,B,D,Cinf,C2,gamma)

% Get Riccati matrices
P1 = reshape(X(1 : 4), 2, 2);
P2 = reshape(X(5 : 8), 2, 2);

% Riccati 1
mat1 = -Cinf' * Cinf - P2 * B * B' * P2 - (1 / gamma^2) * P1 * D * D' * P1 + P1 * A + ...
          + A' * P1 - (P1 * B * B' * P2 + P2 * D * D' * P1);
vec1 = reshape(mat1, 4, 1);
out(1 : 4) = vec1;

% Riccati 2
mat2 = C2' * C2 - P2 * B * B' * P2 + P2 * A + A' * P2 + ...
          - (1 / gamma^2) * (P2 * D * D' * P1 + P1 * D * D' * P2);
vec2 = reshape(mat2, 4, 1);
out(5 : 8) = vec2;
% 
% % Symmetry
% out(9)  = X(2) - X(3);
% out(10) = X(6) - X(7); 


end

    