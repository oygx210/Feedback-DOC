function D = computeParameterLinearization(x, u, p, UQ)

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
hbar   = UQ(11);

% Compute analytical Jacobian
df1dH   = 0;
df2dH   = 0;
df3dH   = 0;
df4dH   = -Dbar .* hbar ./ (mass .* Hbar .^ 2);
df5dH   = Lbar .* cos(bank) .* hbar ./ (mass .* vbar .* Hbar .^ 2);
df6dH   = Lbar .* sin(bank) .* hbar ./ (mass .* vbar .* cos(fpa) .* Hbar .^ 2);
df1dCD0 = 0;
df2dCD0 = 0;
df3dCD0 = 0;
df4dCD0 = qbar .* Sbar ./ mass;
df5dCD0 = 0;
df6dCD0 = 0;

dfdCD0  = [df1dCD0; df2dCD0; df3dCD0; df4dCD0; df5dCD0; df6dCD0];
dfdH    = [df1dH; df2dH; df3dH; df4dH; df5dH; df6dH];
     
D       = [dfdCD0 dfdH];
     
end