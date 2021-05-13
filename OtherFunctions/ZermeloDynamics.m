function xdot = ZermeloDynamics(t,x,ts,us,p)

% Get state at time t
x1 = x(1);
x2 = x(2);

% Interpolate control at time t
u = interp1(ts, us, t);

% Compute derivatives
x1dot   = cos(u) + p * x2;
x2dot   = sin(u);

% Store derivatives
xdot = [x1dot; x2dot];

end