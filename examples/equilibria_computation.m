% Create a feather satellite and the satellite from the paper:
% Then we will run a rootfinding algorithm to determine any equilibria.

l = sym('l', 'real');
w = sym('w', 'real');
h = sym('h', 'real');
d = sym('d', 'real');
l_w = sym('l_w', 'real');
w_w = sym('w_w', 'real');

%Define the satellite bus as a predefined box
bus = saero.geometry.shapes.Box(l, w, h, [0;0;0]);


% Define satellite wings 
cop_wings = [0, 0, 0, 0;
            -d, 0, d, 0;
             0, d, 0, -d];
%Symbolic wing area for each panel
wing_areas = l_w*w_w.*ones(1,4);

%Normals suited to feather satellite
normals_wings_feather = [0, 0, 0, 0;
                        0, 1, 0, -1;
                        1, 0, -1, 0];

%Normals suited to satellite from paper
normals_wings_paper = [1,  1,  1, 1;
                       0,  0,  0, 0;
                       0,  0,  0, 0];
%Define wings as a panel group each
wings_feather = saero.geometry.PanelGroup(cop_wings, normals_wings_feather, wing_areas);
wings_paper = saero.geometry.PanelGroup(cop_wings, normals_wings_paper, wing_areas);

%Full geometries
sat_feather_geometry = saero.geometry.SatelliteGeometry([bus; wings_feather]);
sat_paper_geometry = saero.geometry.SatelliteGeometry([bus; wings_paper]);

%Calculation method
aero = saero.aerodynamics.Sentman();

%Full satellites
sat_feather = saero.Satellite( ...
    "calculation_model",aero,"satellite_geometry", sat_feather_geometry);

sat_paper = saero.Satellite(...
    "calculation_model",aero, "satellite_geometry",sat_paper_geometry);

incoming_velocity = sym("vi", [3,1]);
forceExpr_feather = sat_feather.get_total_aerodynamic_force(incoming_velocity);
torqueExpr_feather = sat_feather.get_total_aerodynamic_torque(incoming_velocity);

forceExpr_paper = sat_paper.get_total_aerodynamic_force(incoming_velocity);
torqueExpr_paper = sat_paper.get_total_aerodynamic_torque(incoming_velocity);

% % Set initial conditions for the rootfinding algorithm
% initialGuessFeather = [0; 0; 0];
% initialGuessPaper = [0; 0; 0];
% % Run the rootfinding algorithm to determine equilibria
% equilibriaFeather = sat_feather.findEquilibria();
% equilibriaPaper = sat_paper.findEquilibria();