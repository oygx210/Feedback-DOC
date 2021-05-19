function output = Zermelo_DOC_Endpoint(input)

% Constants
alpha = input.auxdata.alpha;
SigmaP = input.auxdata.SigmaP;
Qf = input.auxdata.Qf;

% Compute augmented terminal cost
Svecf = input.phase.finalstate(3:6);
Sf = reshape(Svecf, 2, 2);
J_f = 0.5 * trace(Qf * Sf * SigmaP * Sf');

% Compute regular terminal cost
x1f  = input.phase.finalstate(1);
J_0 = -x1f;

output.objective = J_0 + alpha * (J_f + input.phase.integral);

end