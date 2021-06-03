function xdot = ZermeloFeedbackDynamics_v2(t,x,feedback,C,p,num)

% Get state at time t
x1 = x(1);
x2 = x(2);

% Get parameters
p1 = p(1);
p2 = p(2);

% Extract open loop (desensitized control)
ts = feedback.phase.time;
us = feedback.phase.control;
xs = feedback.phase.state(:, 1:2);
if num == 1
    Ps = feedback.phase.state(:, 7:10);
else
    Ps = feedback.phase.state(:, 11 : 14);
end

% Interpolate nominal state, control, and Riccati matrix at time t
x_nom = interp1(ts, xs, t)';
u_nom = interp1(ts, us, t)';
Pvec  = interp1(ts, Ps, t);
P     = reshape(Pvec, 2, 2);

% Compute linearizations at time t
B = computeControlLinearization(x_nom, u_nom, p);

% Compute Kalman gain at time t
K = -inv(C.R) * B' * P;

% Compute feedback control
delta_x = x - x_nom;
delta_u = K * delta_x;

% Combine OL and feedback control
u = u_nom + delta_u;

% Compute derivatives
x1dot   = cos(u) + p1 * x2;
x2dot   = sin(u) + p2;

% Store derivatives
xdot = [x1dot; x2dot];

end