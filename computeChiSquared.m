function x = computeChiSquared(conf,guess)

confk = gammainc(guess/2,1);
tol = 1E-06;
dg = 1E-05;
error_conf = abs(conf - confk);
error_plot = error_conf;
iter = 0;

while error_conf > tol
    if confk > conf
        guess = guess - dg;
        confk = gammainc(guess/2,1);
        error_conf = abs(conf - confk);
        error_plot = [error_plot; error_conf];
        iter = iter + 1;
    else
        guess = guess + dg;
        confk = gammainc(guess/2,1);
        error_conf = abs(conf - confk);
        error_plot = [error_plot; error_conf];
        iter = iter + 1;
    end
end

x = guess;


end