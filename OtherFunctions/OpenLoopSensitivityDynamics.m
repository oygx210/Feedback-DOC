function Sdot = OpenLoopSensitivityDynamics(N,S1,S2,x1,x2,u,p)

% Initialize sensitivity dynamics
Sdot = zeros(N, 2);

for k = 1 : N
    
    % Get sensitivity matrix at time tk
    S1_k = S1(k);
    S2_k = S2(k);
    S_k = [S1_k; S2_k];
    
    % Get state at time tk
    x1_k = x1(k);
    x2_k = x2(k);
    x_k = [x1_k; x2_k];
    
    % Get control at time tk
    u_k = u(k);
    
    % Compute linearizations at time tk
    A = computeStateLinearization(x_k, u_k, p);
    D = computeParameterLinearization(x_k, u_k, p);
    
    % Propagate dynamics
    Sdot_k = A * S_k + D;
    
    % Store derivative
    Sdot(k, :) = Sdot_k';

end

end