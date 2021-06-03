function sol = obtain_FDOC_solution_mixed(C, IC, FC, LB, UB)

% Auxiliary data for GPOPS-II

auxdata.gamma    = C.gamma;
auxdata.p1       = C.p1;
auxdata.p2       = C.p2;
auxdata.SigmaP   = C.SigmaP;
auxdata.alpha    = C.alpha;
auxdata.C2       = C.C2;
auxdata.D2       = C.D2;
auxdata.Cinf     = C.Cinf;
auxdata.Dinf     = C.Dinf;

%-------------------------------------------------------------------%
%--------------- Set Up Problem Using Data Provided Above ----------%
%-------------------------------------------------------------------%

% Time bounds
bounds.phase.initialtime.lower = IC.time;
bounds.phase.initialtime.upper = IC.time;
bounds.phase.finaltime.lower   = LB.tf;
bounds.phase.finaltime.upper   = UB.tf;

% Initial state bounds
bounds.phase.initialstate.lower = [IC.x1, IC.x2, IC.Svec];
bounds.phase.initialstate.upper = [IC.x1, IC.x2, IC.Svec];

% State bounds
bounds.phase.state.lower = [LB.x1, LB.x2, LB.Svec];
bounds.phase.state.upper = [UB.x1, UB.x2, UB.Svec];

% Final state bounds
bounds.phase.finalstate.lower = [LB.x1, FC.x2, LB.Svec];
bounds.phase.finalstate.upper = [UB.x1, FC.x2, UB.Svec];

% Control bounds
bounds.phase.control.lower = LB.u;
bounds.phase.control.upper = UB.u;

% Integral bounds
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1E6;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

% Component-wise guesses
tGuess     = [0; 1];
x1Guess    = [IC.x1; UB.x1];
x2Guess    = [IC.x2; FC.x2];
SvecGuess  = [IC.Svec; IC.Svec];
uGuess     = [UB.u; LB.u];

% Guesses for staste, control, and time
guess.phase.state     = [x1Guess, x2Guess, SvecGuess];
guess.phase.control   = uGuess;
guess.phase.time      = tGuess;
guess.phase.integral  = 10;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%

mesh.method       = 'hp-LiuRao-Legendre';
% mesh.method       = 'hp-LiuRao';
% mesh.method       = 'hp-DarbyRao';
% mesh.method       = 'hp-PattersonRao';
mesh.maxiterations = 30;
mesh.colpointsmin  = 5;
mesh.colpointsmax  = 20;
mesh.tolerance     = 1e-3;

%-------------------------------------------------------------------%
%---------- Configure Setup Using the information provided ---------%
%-------------------------------------------------------------------%
setup.name                           = 'Zermelo-FDOC-Problem-mixed';
setup.functions.continuous           = @Zermelo_FDOC_Continuous_mixed;
setup.functions.endpoint             = @Zermelo_FDOC_Endpoint_mixed;
setup.auxdata                        = auxdata;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.mesh                           = mesh;
setup.displaylevel                   = 2;
setup.nlp.solver                     = 'ipopt';
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.derivatives.supplier           = 'sparseCD';
setup.derivatives.derivativelevel    = 'second';
setup.scales.method                  = 'automatic-bounds';
setup.method                         = 'RPM-Differentiation';

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output   = gpops2(setup);
sol      = output.result.solution;


end