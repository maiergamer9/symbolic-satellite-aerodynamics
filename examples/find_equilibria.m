%% Equilibria Finding Script using CasADi
% Find equilibrium points where τ(vi) = 0 with ||vi|| = 1

% Clear workspace
clear; clc;

% Add CasADi to path
import casadi.*

% Run the movable panels setup
run('movable_panels.m');

%% Satellite parameters
params = struct();
params.l = 0.4;      % Body length [m]
params.w = 0.4;      % Body width [m] 
params.h = 0.4;      % Body height [m]
params.d = 0.45;     % Panel offset distance [m]
params.l_w = 0.275;  % Panel length [m]
params.w_w = 0.39;   % Panel width [m]

%% Create MATLAB function handle from symbolic expression
try
    torque_func = matlabFunction(torqueExpr_paper_movable, 'Vars', ...
        {incoming_velocity, eta1, eta2, eta3, eta4, l, w, h, d, l_w, w_w});
catch ME
    return;
end

%% Setup CasADi optimization problem
% Define CasADi variables
vi = SX.sym('vi', 3, 1);        % Incoming velocity direction
angles = SX.sym('angles', 4, 1); % Panel angles [eta1, eta2, eta3, eta4]

% Casadi Function  (funktioniert noch nicht richtig)
torque_casadi = Function('torque', {vi, angles(1), angles(2), angles(3), angles(4)}, ...
                        {torque_func(vi, angles(1), angles(2), angles(3), angles(4), ...
                         params.l, params.w, params.h, params.d, params.l_w, params.w_w)});

% Define the equilibrium conditions
torque_val = torque_casadi(vi, angles(1), angles(2), angles(3), angles(4));
unit_constraint = vi'*vi - 1;  % ||vi|| = 1 constraint

% Formulate as root-finding problem
g = [torque_val; unit_constraint];  
rootfind_func = Function('equilibrium', {vi, angles}, {g});
rootfinder = rootfinder('newton_solver', 'newton', rootfind_func);
