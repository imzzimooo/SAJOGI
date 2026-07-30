function buildQuadrupedSimulinkModel()
%buildQuadrupedSimulinkModel Create a basic Simulink model for the quadruped controller.
%
% Usage:
%   cd('simulation');
%   buildQuadrupedSimulinkModel();
%
% Requirements:
%   MATLAB with Simulink installed.

    if ~exist('simulink', 'file')
        error('Simulink is not available in the current MATLAB installation.');
    end

    rootDir = fileparts(fileparts(mfilename('fullpath')));
    if ~contains(path, rootDir)
        addpath(genpath(rootDir));
    end

    robot = RobotParameters();
    bodyHeight = robot.BodyHeight;

    modelName = 'quadruped_simulink_model';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    if exist([modelName '.slx'], 'file')
        delete([modelName '.slx']);
    end

    new_system(modelName);
    open_system(modelName);

    add_block('built-in/Clock', [modelName '/Time']);
    add_block('built-in/Constant', [modelName '/BodyPosition']);
    add_block('built-in/Constant', [modelName '/BodyVelocity']);
    add_block('built-in/Constant', [modelName '/BodyRPY']);

    set_param([modelName '/BodyPosition'], 'Value', sprintf('[0 0 %.3f]', bodyHeight));
    set_param([modelName '/BodyVelocity'], 'Value', '[0.2 0 0]');
    set_param([modelName '/BodyRPY'], 'Value', '[0 0 0]');

    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/QuadrupedControllerBlock']);
    set_param([modelName '/QuadrupedControllerBlock'], 'FunctionName', 'QuadrupedSimulinkFcn');

    add_block('simulink/Sinks/To Workspace', [modelName '/JointAnglesOut']);
    add_block('simulink/Sinks/To Workspace', [modelName '/FootWorldOut']);
    add_block('simulink/Sinks/To Workspace', [modelName '/LandingPointsOut']);
    add_block('simulink/Sinks/To Workspace', [modelName '/PhaseInfoOut']);
    add_block('simulink/Sinks/To Workspace', [modelName '/HipPointsOut']);

    set_param([modelName '/JointAnglesOut'], 'VariableName', 'jointAnglesOut');
    set_param([modelName '/FootWorldOut'], 'VariableName', 'footWorldOut');
    set_param([modelName '/LandingPointsOut'], 'VariableName', 'landingPointsOut');
    set_param([modelName '/PhaseInfoOut'], 'VariableName', 'phaseInfoOut');
    set_param([modelName '/HipPointsOut'], 'VariableName', 'hipPointsOut');
    set_param([modelName '/JointAnglesOut'], 'SaveFormat', 'Array');
    set_param([modelName '/FootWorldOut'], 'SaveFormat', 'Array');
    set_param([modelName '/LandingPointsOut'], 'SaveFormat', 'Array');
    set_param([modelName '/PhaseInfoOut'], 'SaveFormat', 'Array');
    set_param([modelName '/HipPointsOut'], 'SaveFormat', 'Array');

    add_line(modelName, 'Time/1', 'QuadrupedControllerBlock/1');
    add_line(modelName, 'BodyPosition/1', 'QuadrupedControllerBlock/2');
    add_line(modelName, 'BodyVelocity/1', 'QuadrupedControllerBlock/3');
    add_line(modelName, 'BodyRPY/1', 'QuadrupedControllerBlock/4');

    add_line(modelName, 'QuadrupedControllerBlock/1', 'JointAnglesOut/1');
    add_line(modelName, 'QuadrupedControllerBlock/2', 'FootWorldOut/1');
    add_line(modelName, 'QuadrupedControllerBlock/3', 'LandingPointsOut/1');
    add_line(modelName, 'QuadrupedControllerBlock/4', 'PhaseInfoOut/1');
    add_line(modelName, 'QuadrupedControllerBlock/5', 'HipPointsOut/1');

    set_param(modelName, 'StopTime', '1');
    set_param(modelName, 'Solver', 'ode45');
    set_param(modelName, 'MaxStep', '0.02');

    save_system(modelName);
    fprintf('Created Simulink model: %s.slx\n', modelName);
    open_system(modelName);
end
