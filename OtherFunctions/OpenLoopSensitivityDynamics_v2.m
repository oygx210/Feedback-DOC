function Sdot = OpenLoopSensitivityDynamics_v2(N,Svec,x1,x2,u,p)

% Initialize sensitivity dynamics
Sdot = zeros(N, 4);

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
    A = computeStateLinearization_v2(x_k, u_k, p);
    D = computeParameterLinearization_v2(x_k, u_k, p);
    
    % Propagate dynamics
    Sdot_k = A * S_k + D;
    
    % Reshape sensitivity matrix -> sensitivity vector
    Sdotvec_k = reshape(Sdot_k, 1, 4);
    
    % Store derivative
    Sdot(k, :) = Sdotvec_k;

end

end