%% Equilibrium Analysis with Movable Panel
% This script demonstrates how to use the movable panel satellite for 
% equilibrium analysis and optimization

run('movable_panels.m');

%% Example 1: Equilibrium condition as symbolic equation


constraint = eta1 + eta3 == 0;
equilibrium_equations = [
    torqueExpr_paper_movable(1) == 0;  % τx = 0
    torqueExpr_paper_movable(2) == 0;  % τy = 0  
    torqueExpr_paper_movable(3) == 0   % τz = 0
    constraint
];


solution = solve(equilibrium_equations, [eta1,eta2,eta3,eta4]);
disp(solution);