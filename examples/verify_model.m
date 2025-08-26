% Define parameter values
d_val = 0.45;
eta1_val = deg2rad(-40);
eta2_val = deg2rad(-40); 
eta3_val = deg2rad(-40);
eta4_val = deg2rad(40);
h_val = 0.4;
l_val = 0.4;
l_w_val = 0.39;
w_val = 0.4;
w_w_val = 0.275;

% Your mustBeZeroFun only takes vi1, vi2, vi3 as arguments
% So create a simple wrapper
equilibrium_fun = @(vi) mustBeZeroFun(vi(1), vi(2), vi(3));

% Initial guess (should be normalized since constraint is ||vi|| = 1)
vi_initial = [1, 1, 0];

% Solve the equilibrium system
options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-12, 'MaxFunctionEvaluations', 1000);
vi_solution = fsolve(equilibrium_fun, vi_initial, options);

% Verify the solution
residual = equilibrium_fun(vi_solution);
fprintf('Solution: vi = [%.6f, %.6f, %.6f]\n', vi_solution);
fprintf('Velocity magnitude: %.6f\n', norm(vi_solution));
fprintf('Residual norm: %.2e\n', norm(residual));