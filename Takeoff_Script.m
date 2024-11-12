clc;
clear;

function [wingspan, maxAoA, dropHeight] = runTakeoff(wingarea, AR)
    % Environmental Condition
    g_accel = 9.807;
    
    % Geometry Data
    mass = 14000; % [kg]
    wingspan = sqrt(AR * wingarea);
    
    % Thrust
    thrust = 10000 * g_accel; % [Newton]
    
    % Aerodynamic Variables
    alpha_L0 = deg2rad(-2);
    CL_alpha = 2 * pi * AR / (2 + sqrt(AR^2 + 4));
    
    % Assign variables needed by Simulink model to the base workspace
    CD_0 = 0.025;
    eswald = 1.78 * (1.0 - 0.045 * AR^0.68) - 0.64;
    k_drag = 1.0 / (eswald * pi * AR);
    CD_LG = 0.01;
    mu_friction_nobrake = 0.04;
    carrier_vel = 25 * 0.5144444444; % [m/s]

    assignin('base', 'wingarea', wingarea);
    assignin('base', 'g_accel', g_accel);
    assignin('base', 'CD_0', CD_0);
    assignin('base', 'k_drag', k_drag);
    assignin('base', 'CD_LG', CD_LG);
    assignin('base', 'mu_friction_nobrake', mu_friction_nobrake);
    assignin('base', 'carrier_vel', carrier_vel);
    assignin('base', 'mass', mass);
    assignin('base', 'wingspan', wingspan);
    assignin('base', 'thrust', thrust);
    assignin('base', 'alpha_L0', alpha_L0);
    assignin('base', 'CL_alpha', CL_alpha);
    
    %% ====================================================================== 
    % Run Simulink Model
    try
        load_system("Takeoff_Sim.slx");  % Load the Simulink model if not already loaded
        simResult = sim("Takeoff_Sim.slx");
        
        % Extract results from simulation
        maxAoA = max(simResult.alpha);
        dropHeight = min(simResult.y_c);
        
    catch ME
        disp("Error running the Simulink model:");
        disp(ME.message);
        maxAoA = NaN;
        dropHeight = NaN;
    end
end

%% Call the function with example inputs
[wingspan, maxAoA, dropHeight] = runTakeoff(15.873, 10);
disp(['Wingspan: ', num2str(wingspan), ' m']);
disp(['Max AoA: ', num2str(maxAoA), ' deg']);
disp(['Drop Height: ', num2str(dropHeight), ' m']);


%% Optimize wing

% Objective function to minimize wing area
function cost = objectiveFunction(x)
    % Extract wing area and aspect ratio from input vector
    wingarea = x(1);
    
    % Set the cost function as the wing area (minimize wing area)
    cost = wingarea;
    
    % Penalize infeasible solutions with NaN results
    if isnan(cost)
        cost = inf;
    end
end

% Nonlinear constraint function for maxAoA and dropHeight limits
function [c, ceq] = constraintFunction(x)
    % Extract wing area and aspect ratio
    wingarea = x(1);
    AR = x(2);
    
    % Run the takeoff simulation
    [wingspan, maxAoA, dropHeight] = runTakeoff(wingarea, AR);
    
    % Constraint inequalities (c <= 0)
    c = [ wingspan - 19.99999;
          maxAoA - 7.99999;     % maxAoA must be ≤ 8 degrees (converted to radians)
         -dropHeight - 19.99999];       % dropHeight must be ≥ -20 m (converted to positive inequality)
    
    % No equality constraints
    ceq = [];
end

function [state, options, optchanged] = outputFunction(options, state, flag)
    global costHistory wingareaHistory ARHistory;
    optchanged = false;
    
    % Get the minimum cost and corresponding design variables in the current generation
    [minCost, idx] = min(state.Score);
    bestWingArea = state.Population(idx, 1);
    bestAR = state.Population(idx, 2);
    
    % Append the minimum cost and corresponding design variables
    costHistory = [costHistory; minCost];
    wingareaHistory = [wingareaHistory; bestWingArea];
    ARHistory = [ARHistory; bestAR];
    
    % Display the current generation's minimum cost and design variables
    disp(['Generation ', num2str(state.Generation), ...
          ': Min Cost = ', num2str(minCost), ...
          ', Wing Area = ', num2str(bestWingArea), ...
          ', Aspect Ratio = ', num2str(bestAR)]);
end

% Set initial guess
wingarea_init = 15; % Initial guess for wing area
AR_init = 5;        % Initial guess for aspect ratio
x0 = [wingarea_init, AR_init];

% Set bounds for wingarea and AR
lb = [1, 1];  % Lower bounds for wingarea and AR
ub = [50, 10]; % Upper bounds for wingarea and AR

% Enable the parallel pool if not already running
if isempty(gcp('nocreate'))
    parpool;  % Start a parallel pool with the default number of workers
end

% Genetic algorithm options with parallel enabled
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', 20, ...
    'MaxGenerations', 50, ...
    'FunctionTolerance', 1e-6, ...
    'Display', 'iter', ...
    'UseParallel', true);  % Enable parallel processing

% Run the genetic algorithm with nonlinear constraints
[x_opt, fval] = ga(@objectiveFunction, 2, [], [], [], [], lb, ub, @constraintFunction, options);

% Display the results
optimal_wingarea = x_opt(1);
optimal_AR = x_opt(2);
disp(['Optimal Wing Area: ', num2str(optimal_wingarea), ' m^2']);
disp(['Optimal Aspect Ratio: ', num2str(optimal_AR)]);
disp(['Objective Function Value (Minimized Wing Area): ', num2str(fval)]);
