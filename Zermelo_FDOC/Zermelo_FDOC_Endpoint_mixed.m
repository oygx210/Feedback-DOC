function output = Zermelo_FDOC_Endpoint_mixed(input)

% Constants
alpha  = input.auxdata.alpha;

% Compute regular terminal cost
x1f  = input.phase.finalstate(1);
J_0 = -x1f;

output.objective = J_0 + alpha * input.phase.integral;

end