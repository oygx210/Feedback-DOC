function Pdot = RiccatiMatrixDynamics(N,Pvec,Q,R,x1,x2,u,p)

% Initialize Riccati matrix dynamics
Pdot = zeros(N, 4);

for k = 1 : N
    
    % Get Riccati vector at time tk
    Pvec_k = Pvec(k,:);
    
    % Reshape Riccati vector -> Riccati matrix
    P_k = reshape(Pvec_k, 2, 2);
    
    % Get state at time tk
    x1_k = x1(k);
    x2_k = x2(k);
    x_k = [x1_k; x2_k];
    
    % Get control at time tk
    u_k = u(k);
    
    % Compute linearizations at time tk
    A_k = computeStateLinearization(x_k, u_k, p);
    B_k = computeControlLinearization(x_k, u_k, p);
    
    % Propagate dynamics
    Pdot_k = -(A_k' * P_k + P_k * A_k + Q - P_k * B_k / R * B_k' * P_k);
    
    % Reshape Riccati matrix -> Riccati vector
    Pdotvec_k = reshape(Pdot_k, 1, 4);
    
    % Store derivative
    Pdot(k, :) = Pdotvec_k;

end

end