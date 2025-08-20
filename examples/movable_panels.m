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
% Base center of pressure positions (fixed attachment points)
cop_wings_base = [-l/2, -l/2, -l/2, -l/2;
                 -d, 0, d, 0;
                  0, d, 0, -d];

% Base normals for paper satellite (all initially pointing in +x direction)
normals_base_paper = [1, 1, 1, 1;
                      0, 0, 0, 0;
                      0, 0, 0, 0];


%% Create rotation matrices for each panel
% Panel 1 (left): Rotation around z-axis
R1 = [cos(eta1), -sin(eta1), 0;
      sin(eta1),  cos(eta1), 0;
      0,         0,         1];

% Panel 2 (bottom): Rotation around y-axis
R2 = [cos(eta2), 0, sin(eta2);
      0,         1, 0;
      -sin(eta2), 0, cos(eta2)];

% Panel 3 (right): Rotation around z-axis 
R3 = [cos(eta3), -sin(eta3), 0;
      sin(eta3),  cos(eta3), 0;
      0,         0,         1];
% Panel 4 (top): Rotation around y-axis 
R4 = [cos(eta4), 0, sin(eta4);
      0,         1, 0;
      -sin(eta4), 0, cos(eta4)];

%% Apply rotations to create moveable panel normals
normals_wings_paper_movable = sym(zeros(3,4));
%Calculate the new normals for each panel after rotation
normals_wings_paper_movable(:, 1) = R1 * normals_base_paper(:, 1);
normals_wings_paper_movable(:, 2) = R2 * normals_base_paper(:, 2);
normals_wings_paper_movable(:, 3) = R3 * normals_base_paper(:, 3);
normals_wings_paper_movable(:, 4) = R4 * normals_base_paper(:, 4);

%% Create moveable COP positions
cop_wings_movable = sym(zeros(3,4));
% Calculate the new center of pressure positions for each panel
cop_wings_movable(:, 1) = R1 * cop_wings_base(:, 1);
cop_wings_movable(:, 2) = R2 * cop_wings_base(:, 2); 
cop_wings_movable(:, 3) = R3 * cop_wings_base(:, 3); 
cop_wings_movable(:, 4) = R4 * cop_wings_base(:, 4);

%% Symbolic wing areas
wings_areas = l_w*w_w*ones(1,4);

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

