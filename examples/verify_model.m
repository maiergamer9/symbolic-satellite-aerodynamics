%% Model Verification Script (Simple Terminal Output)
% Verify that our symbolic satellite model matches the paper exactly

% Clear workspace
clear; clc;

% Run the movable panels setup
run('movable_panels.m');

%% Test configuration from paper (Table 3, Case 1)
% Paddle angles: (-40°, -40°, -40°, 40°)
eta1_test = -40 * pi/180;  % Panel 1 (left)
eta2_test = -40 * pi/180;  % Panel 2 (bottom) 
eta3_test = -40 * pi/180;  % Panel 3 (right)
eta4_test = 40 * pi/180;   % Panel 4 (top)

% Test incoming velocity (paper uses orbital velocity direction)
vi_test = [1; 0; 0];  % Along X-axis (orbital velocity direction)

%% Debug: Check what symbolic variables we have
fprintf('=== Debugging Symbolic Variables ===\n');
fprintf('Force expression variables: ');
force_vars = symvar(forceExpr_paper_movable);
disp(force_vars);

fprintf('Torque expression variables: ');
torque_vars = symvar(torqueExpr_paper_movable);
disp(torque_vars);

%% Substitute values step by step
fprintf('\n=== Substituting Values ===\n');

% Start with original expressions
force_expr = forceExpr_paper_movable;
torque_expr = torqueExpr_paper_movable;

% Substitute satellite parameters first
fprintf('Substituting satellite parameters...\n');
force_expr = subs(force_expr, [l, w, h, d, l_w, w_w], [0.4, 0.4, 0.4, 0.45, 0.275, 0.39]);
torque_expr = subs(torque_expr, [l, w, h, d, l_w, w_w], [0.4, 0.4, 0.4, 0.45, 0.275, 0.39]);

% Substitute panel angles
fprintf('Substituting panel angles...\n');
force_expr = subs(force_expr, [eta1, eta2, eta3, eta4], [eta1_test, eta2_test, eta3_test, eta4_test]);
torque_expr = subs(torque_expr, [eta1, eta2, eta3, eta4], [eta1_test, eta2_test, eta3_test, eta4_test]);

% Substitute incoming velocity
fprintf('Substituting incoming velocity...\n');
force_expr = subs(force_expr, incoming_velocity, vi_test);
torque_expr = subs(torque_expr, incoming_velocity, vi_test);

% Try to convert to double
fprintf('\n=== Converting to Double ===\n');
try
    force_test = double(force_expr);
    fprintf('✓ Force conversion successful\n');
catch ME
    fprintf('✗ Force conversion failed: %s\n', ME.message);
    force_test = [NaN; NaN; NaN];
end

try
    torque_test = double(torque_expr);
    fprintf('✓ Torque conversion successful\n');
catch ME
    fprintf('✗ Torque conversion failed: %s\n', ME.message);
    torque_test = [NaN; NaN; NaN];
end

%% Evaluate panel positions and normals for verification
fprintf('\n=== Panel Positions and Normals ===\n');

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
    fprintf('✓ Panel calculations successful\n');
catch ME
    fprintf('✗ Panel calculations failed: %s\n', ME.message);
    panels_ok = false;
end

%% Display results
fprintf('\n=== Model Verification Results ===\n');
fprintf('Test Configuration: Case 1 from Paper Table 3\n');
fprintf('Panel angles: [%.1f°, %.1f°, %.1f°, %.1f°]\n', ...
        eta1_test*180/pi, eta2_test*180/pi, eta3_test*180/pi, eta4_test*180/pi);
fprintf('Incoming velocity: [%.2f, %.2f, %.2f]\n\n', vi_test);

if panels_ok
    fprintf('Panel Positions [m]:\n');
    fprintf('  Panel 1 (left):   [%7.3f, %7.3f, %7.3f]\n', panel_positions(:,1));
    fprintf('  Panel 2 (bottom): [%7.3f, %7.3f, %7.3f]\n', panel_positions(:,2));
    fprintf('  Panel 3 (right):  [%7.3f, %7.3f, %7.3f]\n', panel_positions(:,3));
    fprintf('  Panel 4 (top):    [%7.3f, %7.3f, %7.3f]\n\n', panel_positions(:,4));

    fprintf('Panel Normal Vectors (after rotation):\n');
    fprintf('  Panel 1 (left):   [%7.3f, %7.3f, %7.3f]\n', panel_normals(:,1));
    fprintf('  Panel 2 (bottom): [%7.3f, %7.3f, %7.3f]\n', panel_normals(:,2));
    fprintf('  Panel 3 (right):  [%7.3f, %7.3f, %7.3f]\n', panel_normals(:,3));
    fprintf('  Panel 4 (top):    [%7.3f, %7.3f, %7.3f]\n\n', panel_normals(:,4));
end

if ~any(isnan(force_test))
    fprintf('Aerodynamic Force [N]:\n');
    fprintf('  Fx = %10.6f\n', force_test(1));
    fprintf('  Fy = %10.6f\n', force_test(2));
    fprintf('  Fz = %10.6f\n\n', force_test(3));
end

if ~any(isnan(torque_test))
    fprintf('Aerodynamic Torque [Nm]:\n');
    fprintf('  Tx = %10.6f\n', torque_test(1));
    fprintf('  Ty = %10.6f\n', torque_test(2));
    fprintf('  Tz = %10.6f\n\n', torque_test(3));
end