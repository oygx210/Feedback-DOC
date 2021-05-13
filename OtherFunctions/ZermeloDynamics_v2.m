function xdot = ZermeloDynamics_v2(t,x,ts,us,p)

% Get state at time t
x1 = x(1);
x2 = x(2);

% Get parameters
p1 = p(1);
p2 = p(2);

% Interpolate control at time t
u = interp1(ts, us, t);

% Compute derivatives
x1dot   = cos(u) + p1 * x2;
x2dot   = sin(u) + p2;

% Store derivatives
xdot = [x1dot; x2dot];

end