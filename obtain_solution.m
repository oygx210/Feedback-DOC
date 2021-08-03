function sol = obtain_solution(alpha, C, IC, FC, LB, UB, ND)

%% Auxiliary data for GPOPS-II

auxdata.alpha    = alpha;
auxdata.mubar    = C.mubar;
auxdata.rho0bar  = C.rho0bar;
auxdata.Hbar     = C.Hbar;
auxdata.Sbar     = C.Sbar;

auxdata.CD0      = C.CD0;
auxdata.K        = C.K;
auxdata.N        = C.N;
auxdata.Re       = C.Re;
auxdata.g0       = C.g0;
auxdata.SigmaP   = C.SigmaP;
auxdata.R        = C.R;
auxdata.Qf       = C.Qf;

auxdata.mass     = C.mass;
auxdata.kQ       = C.kQ;

%-------------------------------------------------------------------%
%--------------- Set Up Problem Using Data Provided Above ----------%
%-------------------------------------------------------------------%
% Time bounds
bounds.phase.initialtime.lower = IC.tau;
bounds.phase.initialtime.upper = IC.tau;
bounds.phase.finaltime.lower   = FC.tauf;
bounds.phase.finaltime.upper   = FC.tauf;

% Initial state bounds
bounds.phase.initialstate.lower = [IC.rbar, IC.lon,  IC.lat, IC.vbar, ...
                                   IC.fpa,  IC.azi,  IC.Svec, ...
                                   LB.CL,   LB.bank, LB.Pvec];
bounds.phase.initialstate.upper = [IC.rbar, IC.lon,  IC.lat, IC.vbar, ...
                                   IC.fpa,  IC.azi,  IC.Svec, ...
                                   UB.CL,   UB.bank, UB.Pvec];

% State bounds
bounds.phase.state.lower = [LB.rbar, LB.lon,  LB.lat, LB.vbar, ...
                            LB.fpa,  LB.azi,  LB.Svec, ...
                            LB.CL,   LB.bank, LB.Pvec];                        
bounds.phase.state.upper = [UB.rbar, UB.lon,  UB.lat, UB.vbar, ...
                            UB.fpa,  UB.azi,  UB.Svec, ...
                            UB.CL,   UB.bank, UB.Pvec];
                        
% Final state bounds
bounds.phase.finalstate.lower = [FC.rbar,   FC.lon,  FC.lat, FC.vbar, ...
                                 FC.fpaMin, FC.azi,  LB.Svec, ...
                                 LB.CL,     LB.bank, LB.Pvec];
bounds.phase.finalstate.upper = [FC.rbar,   FC.lon,  FC.lat, FC.vbar, ...
                                 FC.fpaMax, FC.azi,  UB.Svec, ...
                                 UB.CL,     UB.bank, UB.Pvec];
                             
% Control bounds
bounds.phase.control.lower = [LB.CLdotbar, LB.bankdotbar];
bounds.phase.control.upper = [UB.CLdotbar, UB.bankdotbar];

% Bounds for constraint functions
bounds.phase.path.lower = zeros(1, 3);
bounds.phase.path.upper = [C.Qdotmax C.qbarmax C.nmax];

% Integral bounds
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1E6;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

% Component-wise guesses
tGuess               = [0; 1.44552];
radGuess             = [IC.rbar; FC.rbar];
lonGuess             = [IC.lon; FC.lon];
latGuess             = [IC.lat; FC.lat];
speedGuess           = [IC.vbar; FC.vbar];
fpaGuess             = [IC.fpa; -0.105];
aziGuess             = [IC.azi; FC.azi];
SvecGuess            = [zeros(1, 12); 2.7,46.9,10.8,112.6,-1380.9,-69.9,...
                        2.7,46.9,10.8,112.6,-1380.9,-69.9];
PvecGuess            = [zeros(1, 36); FC.Pvec];
CLGuess              = [0.3607; 0.7683];  
bankGuess            = [-18.0989*pi/180; 61.3515*pi/180];

% Guesses for state, control and time
guess.phase.state    = [radGuess, lonGuess, latGuess, speedGuess,...
                       fpaGuess, aziGuess, SvecGuess, CLGuess, bankGuess, PvecGuess];
guess.phase.control  = [0.0006   -0.0026; -0.0368   -0.4680];
guess.phase.time     = tGuess;
guess.phase.integral = 1;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%

% if alpha == 0
%     mesh.method       = 'hp-LiuRao-Legendre';
%     % mesh.method       = 'hp-LiuRao';
%     % mesh.method       = 'hp-DarbyRao';
%     % mesh.method       = 'hp-PattersonRao';
%     mesh.maxiterations = 30;
%     mesh.colpointsmin  = 3;
%     mesh.colpointsmax  = 20;
%     mesh.tolerance     = 5e-5;
% else
%     mesh.method       = 'hp-LiuRao-Legendre';
% %     mesh.method       = 'hp-LiuRao';
% %     mesh.method       = 'hp-DarbyRao';
% %     mesh.method       = 'hp-PattersonRao';
%     mesh.maxiterations = 30;
%     mesh.colpointsmin  = 3;
%     mesh.colpointsmax  = 40;
%     mesh.tolerance     = 1e-3;
% end

mesh.method       = 'hp-LiuRao-Legendre';
% mesh.method       = 'hp-LiuRao';
% mesh.method       = 'hp-DarbyRao';
% mesh.method       = 'hp-PattersonRao';
mesh.maxiterations = 30;
mesh.colpointsmin  = 3;
mesh.colpointsmax  = 20;
mesh.tolerance     = 5e-5;

%-------------------------------------------------------------------%
%---------- Configure Setup Using the information provided ---------%
%-------------------------------------------------------------------%
setup.name                           = 'Hypersonic-Problem';
setup.functions.continuous           = @Hypersonic_Continuous;
setup.functions.endpoint             = @Hypersonic_Endpoint;
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
solution = output.result.solution;

%% Reconstruct solution using ode45
sol = reconstruct_solution(solution, C, IC, ND);

end