function output = Zermelo_baseline_Endpoint(input)

x1f  = input.phase.finalstate(1);

output.objective = -x1f;

end