function output = Hypersonic_Endpoint(input)

alpha  = input.auxdata.alpha;
SigmaP = input.auxdata.SigmaP;
Qf     = input.auxdata.Qf;

rbarf  = input.phase.finalstate(1);

Sfvec  = input.phase.finalstate(7 : 18);
Sf     = reshape(Sfvec, 6, 2);

% G      = [0, 1, 0, 0, 0, 0; ...
%           0, 0, 1, 0, 0, 0; ...
%           mubar/rbarf^2, 0, 0, vbarf, 0, 0];

sensitivity_cost = trace(Qf * Sf * SigmaP * Sf');

output.objective = rbarf + alpha * (sensitivity_cost + input.phase.integral);

end