%% Model Verification Script (Simple Terminal Output)
% Verify that our symbolic satellite model matches the paper exactly

% Clear workspace
clear; clc;

% Run the movable panels setup
run('movable_panels.m');

%% Case 1 from paper
% Paddle angles: (-40°, -40°, -40°, 40°)
eta1_test = -40 * pi/180;
eta2_test = -40 * pi/180;   
eta3_test = -40 * pi/180; 
eta4_test = 40 * pi/180;  

% Test incoming velocity (paper uses orbital velocity direction)
vi_test = [1; 0; 0];  % Along X-axis

%% Substitute values step by step

% Start with original expressions
force_expr = forceExpr_paper_movable;
torque_expr = torqueExpr_paper_movable;

% Substitute satellite parameters
force_expr = subs(force_expr, [l, w, h, d, l_w, w_w], [0.4, 0.4, 0.4, 0.45, 0.275, 0.39]);
torque_expr = subs(torque_expr, [l, w, h, d, l_w, w_w], [0.4, 0.4, 0.4, 0.45, 0.275, 0.39]);

% Substitute panel angles
force_expr = subs(force_expr, [eta1, eta2, eta3, eta4], [eta1_test, eta2_test, eta3_test, eta4_test]);
torque_expr = subs(torque_expr, [eta1, eta2, eta3, eta4], [eta1_test, eta2_test, eta3_test, eta4_test]);

% Substitute incoming velocity
force_expr = subs(force_expr, incoming_velocity, vi_test);
torque_expr = subs(torque_expr, incoming_velocity, vi_test);

% Symbolic to numerical (should be faster)
try
    force_test = double(force_expr);
catch ME
    force_test = [NaN; NaN; NaN];
end

try
    torque_test = double(torque_expr);
catch ME
    torque_test = [NaN; NaN; NaN];
end

%% Evaluate panel positions and normals for verification

% Substitute in panel expressions
cop_expr = cop_wings_movable;
normals_expr = normals_wings_paper_movable;

% Same substitution for panels
cop_expr = subs(cop_expr, [l, w, h, d, l_w, w_w], [0.4, 0.4, 0.4, 0.45, 0.275, 0.39]);
cop_expr = subs(cop_expr, [eta1, eta2, eta3, eta4], [eta1_test, eta2_test, eta3_test, eta4_test]);

normals_expr = subs(normals_expr, [l, w, h, d, l_w, w_w], [0.4, 0.4, 0.4, 0.45, 0.275, 0.39]);
normals_expr = subs(normals_expr, [eta1, eta2, eta3, eta4], [eta1_test, eta2_test, eta3_test, eta4_test]);

try
    panel_positions = double(cop_expr);
    panel_normals = double(normals_expr);
    panels_ok = true;
catch ME
    panels_ok = false;
end
