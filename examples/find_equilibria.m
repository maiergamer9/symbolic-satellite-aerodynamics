%% Equilibria Finding Script using CasADi
% Find equilibrium points where τ(vi) = 0 with ||vi|| = 1

% Clear workspace
clear; clc;

% Add CasADi to path (adjust path as needed)
% addpath('path/to/casadi');
import casadi.*

% Run the movable panels setup
run('movable_panels.m');

%% Satellite parameters (from paper Table 1)
params = struct();
params.l = 0.4;      % Body length [m]
params.w = 0.4;      % Body width [m] 
params.h = 0.4;      % Body height [m]
params.d = 0.45;     % Panel offset distance [m]
params.l_w = 0.275;  % Panel length [m]
params.w_w = 0.39;   % Panel width [m]

%% Create MATLAB function handle from symbolic expression
fprintf('Creating MATLAB function from symbolic expression...\n');

% Check what variables are in the torque expression
torque_vars = symvar(torqueExpr_paper_movable);
fprintf('Variables in torque expression: ');
disp(torque_vars);

% Create function handle with ALL variables explicitly listed
% torque = f(vi1, vi2, vi3, eta1, eta2, eta3, eta4, l, w, h, d, l_w, w_w)
try
    torque_func = matlabFunction(torqueExpr_paper_movable, 'Vars', ...
        {incoming_velocity, eta1, eta2, eta3, eta4, l, w, h, d, l_w, w_w});
    fprintf('✓ MATLAB function created successfully\n');
catch ME
    fprintf('✗ MATLAB function creation failed: %s\n', ME.message);
    return;
end

%% Setup CasADi optimization problem
fprintf('Setting up CasADi equilibrium finding...\n');

% Define CasADi variables
vi = SX.sym('vi', 3, 1);        % Incoming velocity direction
angles = SX.sym('angles', 4, 1); % Panel angles [eta1, eta2, eta3, eta4]

% Create CasADi function from MATLAB function handle
torque_casadi = Function('torque', {vi, angles(1), angles(2), angles(3), angles(4)}, ...
                        {torque_func(vi, angles(1), angles(2), angles(3), angles(4), ...
                         params.l, params.w, params.h, params.d, params.l_w, params.w_w)});

% Define the equilibrium conditions
torque_val = torque_casadi(vi, angles(1), angles(2), angles(3), angles(4));
unit_constraint = vi'*vi - 1;  % ||vi|| = 1 constraint

% Formulate as root-finding problem
g = [torque_val; unit_constraint];  % 4 equations (3 torque + 1 constraint)

% Create root-finding function
rootfind_func = Function('equilibrium', {vi, angles}, {g});

% Setup rootfinder (Newton's method)
rootfinder = rootfinder('newton_solver', 'newton', rootfind_func);

%% Test different panel configurations
fprintf('\n=== Finding Equilibria for Different Configurations ===\n');

% Test configurations from paper Table 3
test_configs = [
    -40, -40, -40,  40;   % Case 1: Expected eq at (0°, 29°, 0°)
    -40, -30,  40, -20;   % Case 2: Expected eq at (9°, 7°, 16°)
      0,   0,   0,   0;   % Case 3: Expected eq at (any, 0°, 0°)
    -40, -10,  40, -10;   % Case 4: Expected eq at (0°, 0°, 23°)
     20,  20, -30, -30;   % Case 5: Expected unstable at (-45°, 0°, -67°)
];

config_names = {'Case 1', 'Case 2', 'Case 3', 'Case 4', 'Case 5'};

results = struct();

for i = 1:size(test_configs, 1)
    fprintf('\n--- %s: Panel angles [%.0f°, %.0f°, %.0f°, %.0f°] ---\n', ...
            config_names{i}, test_configs(i,:));
    
    % Convert angles to radians
    angles_rad = test_configs(i,:) * pi/180;
    
    % Try multiple initial guesses for robustness
    initial_guesses = [
        [1; 0; 0];      % X-direction
        [0; 1; 0];      % Y-direction  
        [0; 0; 1];      % Z-direction
        [1; 1; 0]/norm([1; 1; 0]);    % XY-diagonal
        [1; 0; 1]/norm([1; 0; 1]);    % XZ-diagonal
        [0; 1; 1]/norm([0; 1; 1]);    % YZ-diagonal
    ];
    
    equilibria = [];
    
    for j = 1:size(initial_guesses, 2)
        try
            % Solve for equilibrium
            result = rootfinder('x0', initial_guesses(:,j), 'p', angles_rad');
            vi_eq = full(result.x);
            
            % Verify solution
            torque_check = full(torque_casadi(vi_eq, angles_rad(1), angles_rad(2), angles_rad(3), angles_rad(4)));
            constraint_check = norm(vi_eq) - 1;
            
            % Check if this is a new equilibrium
            is_new = true;
            for k = 1:size(equilibria, 2)
                if norm(vi_eq - equilibria(:,k)) < 1e-6
                    is_new = false;
                    break;
                end
            end
            
            % Add if new and valid
            if is_new && norm(torque_check) < 1e-6 && abs(constraint_check) < 1e-6
                equilibria = [equilibria, vi_eq];
                
                % Convert to spherical coordinates for easier interpretation
                [phi, theta, r] = cart2sph(vi_eq(1), vi_eq(2), vi_eq(3));
                phi_deg = phi * 180/pi;      % Azimuth
                theta_deg = theta * 180/pi;  % Elevation
                
                fprintf('  Equilibrium %d: vi = [%.4f, %.4f, %.4f]\n', ...
                        size(equilibria,2), vi_eq(1), vi_eq(2), vi_eq(3));
                fprintf('                 Spherical: φ=%.1f°, θ=%.1f°\n', phi_deg, theta_deg);
                fprintf('                 Torque residual: %.2e\n', norm(torque_check));
            end
            
        catch ME
            % Rootfinder failed for this initial guess
            continue;
        end
    end
    
    % Store results
    results.(sprintf('case_%d', i)) = struct();
    results.(sprintf('case_%d', i)).angles = test_configs(i,:);
    results.(sprintf('case_%d', i)).equilibria = equilibria;
    results.(sprintf('case_%d', i)).num_equilibria = size(equilibria, 2);
    
    if size(equilibria, 2) == 0
        fprintf('  No equilibria found!\n');
    else
        fprintf('  Found %d equilibrium point(s)\n', size(equilibria, 2));
    end
end

%% Visualize results
fprintf('\n=== Visualization ===\n');
visualize_equilibria(results, test_configs, config_names);

%% Save results
save('equilibria_results.mat', 'results', 'test_configs', 'config_names', 'params');
fprintf('\nResults saved to equilibria_results.mat\n');

%% Helper function for visualization
function visualize_equilibria(results, test_configs, config_names)
    figure(2); clf;
    
    % Create sphere for visualization
    [X, Y, Z] = sphere(50);
    
    % Plot unit sphere
    surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1, 'FaceColor', 'cyan');
    hold on; axis equal; grid on;
    
    colors = {'ro', 'go', 'bo', 'mo', 'ko'};
    
    for i = 1:length(config_names)
        field_name = sprintf('case_%d', i);
        if isfield(results, field_name)
            equilibria = results.(field_name).equilibria;
            
            if size(equilibria, 2) > 0
                % Plot equilibria on unit sphere
                for j = 1:size(equilibria, 2)
                    plot3(equilibria(1,j), equilibria(2,j), equilibria(3,j), ...
                          colors{i}, 'MarkerSize', 10, 'LineWidth', 2);
                end
                
                % Add legend entry
                plot3(NaN, NaN, NaN, colors{i}, 'MarkerSize', 10, 'LineWidth', 2, ...
                      'DisplayName', sprintf('%s (%d eq.)', config_names{i}, size(equilibria,2)));
            end
        end
    end
    
    % Add coordinate axes
    quiver3(0,0,0, 1.2,0,0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.1);
    quiver3(0,0,0, 0,1.2,0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.1);
    quiver3(0,0,0, 0,0,1.2, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.1);
    text(1.3, 0, 0, 'X', 'FontSize', 12, 'FontWeight', 'bold');
    text(0, 1.3, 0, 'Y', 'FontSize', 12, 'FontWeight', 'bold');
    text(0, 0, 1.3, 'Z', 'FontSize', 12, 'FontWeight', 'bold');
    
    xlabel('vi_x'); ylabel('vi_y'); zlabel('vi_z');
    title('Equilibrium Points on Unit Sphere');
    legend('Location', 'best');
    view(45, 30);
end