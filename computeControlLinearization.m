function B = computeControlLinearization(x, u, p, UQ)

% State, control, and parameters
rbar  = x(1);
lon   = x(2);
lat   = x(3);
vbar  = x(4);
fpa   = x(5);
azi   = x(6);

CL    = u(1);
bank  = u(2);

CD0   = p(1);
Hbar  = p(2);

% Useful quantities
londot = UQ(1);
latdot = UQ(2);
qbar   = UQ(3);
Dbar   = UQ(4);
Lbar   = UQ(5);
mass   = UQ(6);
mubar  = UQ(7);
K      = UQ(8);
N      = UQ(9);
Sbar   = UQ(10);

% Compute analytical Jacobian
df1du = [0 0];
df2du = [0 0];
df3du = [0 0];
df4du = [-Lbar / mass * K * N * CL ^ (N - 2), 0];
df5du = [qbar * Sbar / (mass * vbar) * cos(bank), ...
         -Lbar / (mass * vbar) * sin(bank)];
df6du = [qbar * Sbar / (mass * vbar * cos(fpa)), ...
         Lbar / (mass * vbar * cos(fpa)) * cos(bank)];

     
B     = [df1du; df2du; df3du; df4du; df5du; df6du];
     
end