function phaseout = Hypersonic_Continuous(input)

% ---------------------------------------------------%
% ------ Extract Each Component of the State ------- %
% ---------------------------------------------------%
rbar  = input.phase.state(:,1);
lon   = input.phase.state(:,2);
lat   = input.phase.state(:,3);
vbar  = input.phase.state(:,4);
fpa   = input.phase.state(:,5);
azi   = input.phase.state(:,6);

u1dot = input.phase.control(:,1);
u2dot = input.phase.control(:,2);

Svec = input.phase.state(:, 7 : 18);
Pvec = input.phase.state(:, 21 : 56);

CL    = input.phase.state(:, 19);
bank  = input.phase.state(:, 20);

L     = length(rbar);

% ---------------------------------------------------%
% ----------------- Auxiliary data ------------------%
% ---------------------------------------------------%
mubar    = input.auxdata.mubar;
rho0bar  = input.auxdata.rho0bar;
Hbar     = input.auxdata.Hbar;
Sbar     = input.auxdata.Sbar;

CD0      = input.auxdata.CD0;
K        = input.auxdata.K;
N        = input.auxdata.N;
Re       = input.auxdata.Re;
g0       = input.auxdata.g0;

mass     = input.auxdata.mass;
kQ       = input.auxdata.kQ;

p        = [CD0; Hbar];
R        = input.auxdata.R;
SigmaP   = input.auxdata.SigmaP;

% Some calculations
hbar     = (rbar - 1);
CD       = CD0 + real(K * CL .^ N);

rhobar   = rho0bar * exp(-hbar / Hbar);
qbar     = 0.5 * rhobar .* vbar .^ 2;
Dbar     = qbar .* Sbar .* CD;
Lbar     = qbar .* Sbar .* CL;

% ---------------------------------------------------%
% ---- Evaluate Right-Hand Side of the Dynamics ---- %
% ---------------------------------------------------%
raddot = vbar .* sin(fpa);
londot = vbar .* cos(fpa) .* sin(azi) ./ (rbar .* cos (lat));
latdot = vbar.*cos(fpa).*cos(azi)./rbar;
vdot   = -Dbar./mass - mubar.*sin(fpa)./rbar.^2;
fpadot = Lbar.*cos(bank)./(mass.*vbar) - ...
            mubar.*cos(fpa)./(rbar.^2 .* vbar) + vbar.*cos(fpa)./rbar;
azidot = Lbar.*sin(bank)./(mass.*vbar.*cos(fpa)) + ...
                            vbar.*cos(fpa).*sin(azi).*tan(lat)./rbar;

CLdot   = u1dot;
bankdot = u2dot;


% -----------------------------------------------------------------%
% ---------------------- Evaluate Feedback Cost ------------------ %
% -----------------------------------------------------------------%

% Initialize Riccati matrix dynamics
Pdot = zeros(L, 36);

% Initialize sensitivity matrix dynamics
Sdot = zeros(L, 12);

% Initialize feedback cost
J_feedback = zeros(L, 1);

for k = 1 : L % Loop through each time step 
    
    % Get Riccati vector at time tk
    Pvec_k = Pvec(k, :);
    
    % Reshape Riccati vector -> Riccati matrix
    P_k = reshape(Pvec_k, 6, 6);
    
    % Get sensitivity vector at time tk
    Svec_k = Svec(k, :);
    
    % Reshape sensitivity vector -> sensitivity matrix
    S_k = reshape(Svec_k, 6, 2);
    
    % Get state at time tk
    x_k = [rbar(k); lon(k); lat(k); vbar(k); fpa(k); azi(k)];
    
    % Get control at time tk
    u_k = [CL(k); bank(k)];
    
    % Compute linearizations at time tk
    UQ_k = [londot(k); latdot(k); qbar(k); 
            Dbar(k); Lbar(k); mass; mubar;
            K; N; Sbar; hbar]; % Useful quantities
    A_k = computeStateLinearization(x_k, u_k, p, UQ_k);
    B_k = computeControlLinearization(x_k, u_k, p, UQ_k);
    D_k = computeParameterLinearization(x_k, u_k, p, UQ_k);
    
    % Compute Kalman gain
    K_k = -R \ B_k' * P_k;
    
    % Propagate sensitivity dynamics
    Sdot_k = (A_k + B_k * K_k) * S_k + D_k;
    
    % Reshape sensitivity matrix -> sensitivity vector
    Sdotvec_k = reshape(Sdot_k, 1, 12);
    
    % Propagate Riccati dynamics
    Pdot_k = -(A_k' * P_k + P_k * A_k - P_k * B_k / R * B_k' * P_k);
    
    % Reshape Riccati matrix -> Riccati vector
    Pdotvec_k = reshape(Pdot_k, 1, 36);
    
    % Store derivatives
    Sdot(k, :) = Sdotvec_k;
    Pdot(k, :) = Pdotvec_k;
    
    % Store running cost
    J_feedback(k) = trace(R * K_k * S_k * SigmaP * S_k' * K_k');

end

% ---------------------------------------------------%
% --------------- Path Constraints ----------------- %
% ---------------------------------------------------%

heating_rate     = kQ * sqrt(g0)^3 * sqrt(rhobar) .* vbar.^3 ;
dynamic_pressure = qbar .* g0 / Re^2;
normal_load      = sqrt(Lbar.^2 + Dbar.^2) * g0 / mass;

% Output dynamics, path constraints, and running cost
phaseout.dynamics  = [raddot, londot, latdot, vdot, fpadot, azidot, ...
                      Sdot, CLdot, bankdot, Pdot];
phaseout.path      = [heating_rate, dynamic_pressure, normal_load];
phaseout.integrand = J_feedback;

    