function solution  = mixed_L2_Linf_LMI(A,B,D,C2,D2,Cinf,Dinf,gamma)

n     = 2;
m     = 1;
r     = 2;
ell   = n + m;

% Define optimization variables
lambda = sdpvar(1);
%gamma  = sdpvar(1);
X      = sdpvar(n + m, n + m);
Y      = sdpvar(n, n);
Z      = sdpvar(m, n);

% Define objective function
Objective = lambda;

% Define constraints
H2_costConstraint1  = lambda - trace(X) >= 0;
H2_costConstraint2  = [X                    C2 * Y + D2 * Z;
                       (C2 * Y + D2 * Z)'   Y              ] >= 0;
% Hinf_normConstraint = [-Y             zeros(n, n)                        zeros(n, r)      zeros(n, ell)         ;
%                        zeros(n, n)    Y * A' + A * Y + Z' * B' + B * Z   D                Y * Cinf' + Z' * Dinf';
%                        zeros(r, n)    D'                                 -gamma * eye(r)  zeros(r, ell)         ;
%                        zeros(ell, n)  Cinf * Y + Dinf * Z                zeros(ell, r)    -gamma * eye(ell)     ] <= 0;
Q_YZ                = -A * Y - Y * A' - B * Z - Z' * B' - D * D';
Hinf_normConstraint = [Q_YZ (Cinf * Y + Dinf * Z)'; Cinf * Y + Dinf * Z gamma^2 * eye(ell)] >= 0;
Constraints         = [H2_costConstraint1, H2_costConstraint2, Hinf_normConstraint]; 

% Solve the Problem
options   = sdpsettings('solver', 'mosek');
sol       = optimize(Constraints, Objective, options);

% Get performance measure values
time      = sol.solvertime;
obj       = value(Objective);

% Get control values
%gammasol   = value(gamma);
lambdasol  = value(lambda);
Xsol       = value(X);
Ysol       = value(Y);
Zsol       = value(Z);

% Store output
solution.simTime = time;
solution.obj     = obj;
%solution.gamma   = gammasol;
solution.lambda  = lambdasol;
solution.X       = Xsol;
solution.Y       = Ysol;
solution.Z       = Zsol;

end