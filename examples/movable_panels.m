%Enhanced equilibria computation with moveable panels
%This is done by introducing symbolic angles n1, n2, n3, n4 for
%controllable panel orientations

%% Define symbolic variables
l = sym('l', 'real');
w = sym('w', 'real');
h = sym('h', 'real');
d = sym('d', 'real');
l_w = sym('l_w', 'real');
w_w = sym('w_w', 'real');

% Define symbolic panel angles (in radians)
eta1 = sym('eta1', 'real');
eta2 = sym('eta2', 'real');
eta3 = sym('eta3', 'real');
eta4 = sym('eta4', 'real');

%% Define the satellite bus as a predefined box
bus = saero.geometry.shapes.Box(l, w, h, [0;0;0]);

%% Wing panel base configuration
normals_wings_paper_movable = [cos(eta1) , cos(eta2), cos(eta3), cos(eta4) ; ...
                               -sin(eta1), 0        , sin(eta3), 0         ; ...
                               0         ,-sin(eta2), 0        , sin(eta4)];

cop_wings_movable = [-l/2*cos(eta1) + (l/2 + d + l_w /2)*sin(eta1),-l/2*cos(eta2) - (l/2 + d + l_w /2)*sin(eta2),-l/2*cos(eta3) - (l/2 + d + l_w /2)*sin(eta3) , -l/2*cos(eta4) + (l/2 + d + l_w /2)*sin(eta4)  ; ...
                     -(l/2 + d + l_w /2)*cos(eta1) - l/2*sin(eta1), 0                                           ,  (l/2 + d + l_w /2)*cos(eta3) - l/2*sin(eta3),  0                                             ; ...
                     0                                            , (l/2 + d + l_w /2)*cos(eta2) - l/2*sin(eta2),  0                                           , -(l/2 + d + l_w /2)*cos(eta4) - l/2*sin(eta4) ];
% alt
% cop_wings_movable = [  d*sin(eta1) - (l*cos(eta1))/2, d*sin(eta2) - (l*cos(eta2))/2, - (l*cos(eta3))/2 - d*sin(eta3), - (l*cos(eta4))/2 - d*sin(eta4); ...
%                       - d*cos(eta1) - (l*sin(eta1))/2,                             0,   d*cos(eta3) - (l*sin(eta3))/2,                              0; ...
%                               0, d*cos(eta2) + (l*sin(eta2))/2,                               0,   (l*sin(eta4))/2 - d*cos(eta4)];

%% Symbolic wing areas
wings_areas = w_w*l_w*ones(1,4);

%% Create panel groups with moveable panels
wings_paper_movable = saero.geometry.PanelGroup(cop_wings_movable, ...
    normals_wings_paper_movable, wings_areas);

%% Create satellite geometry
sat_paper_geometry_movable = saero.geometry.SatelliteGeometry([bus;wings_paper_movable]);

%% Create aerodynamic calculation method
aero = saero.aerodynamics.Sentman();

%% Create satellite with movable panels
sat_paper_movable = saero.Satellite("calculation_model",aero, ...
    "satellite_geometry", sat_paper_geometry_movable);
%% Calculate forces and torques as symbolic expressions
incoming_velocity = sym("vi", [3,1]);

% Forces and torques
forceExpr_paper_movable = sat_paper_movable.get_total_aerodynamic_force(incoming_velocity);
torqueExpr_paper_movable = sat_paper_movable.get_total_aerodynamic_torque(incoming_velocity);

%% Create MATLAB function handles for numerical evaluation
forceFun_paper = matlabFunction(forceExpr_paper_movable);
torqueFun_paper = matlabFunction(torqueExpr_paper_movable);

%% Define parameter values
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

%% Create equilibrium system
vi = incoming_velocity;
torque0 = torqueFun_paper(d_val, eta1_val, eta2_val, eta3_val, eta4_val, ...
                         h_val, l_val, l_w_val, vi(1), vi(2), vi(3), w_val, w_w_val);
mustBeZero = [torque0; incoming_velocity'*incoming_velocity-1];
mustBeZeroFun = matlabFunction(mustBeZero, 'Vars', {vi(1), vi(2), vi(3)});

%% Solve the equilibrium system using fsolve
equilibrium_fun = @(v) mustBeZeroFun(v(1), v(2), v(3));
vi_initial = [1, 0, 0];  % Initial guess
options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-12);
v0 = fsolve(equilibrium_fun, vi_initial, options);

%% Display results
fprintf('Equilibrium velocity: [%.6f, %.6f, %.6f]\n', v0);
fprintf('Velocity magnitude: %.6f\n', norm(v0));
residual = equilibrium_fun(v0);
fprintf('Residual norm: %.2e\n', norm(residual));