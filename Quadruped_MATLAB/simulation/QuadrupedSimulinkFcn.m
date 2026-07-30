function [jointAnglesOut, footWorldOut, landingPointsOut, phaseInfoOut, hipPointsOut] = QuadrupedSimulinkFcn(timeIn, bodyPositionIn, bodyVelocityIn, bodyRPYIn)
% QuadrupedSimulinkFcn
% MATLAB Function block wrapper for the quadruped controller.
% This block can be used inside a Simulink model to drive the existing
% MATLAB controller implementation.

    coder.extrinsic('RobotParameters', 'TerrainParameters', 'GaitParameters', 'QuadrupedController');

    persistent controller;
    persistent initialized;

    if isempty(initialized)
        rootDir = fileparts(fileparts(mfilename('fullpath')));
        if ~contains(path, rootDir)
            addpath(genpath(rootDir));
        end

        robot = RobotParameters();
        terrain = TerrainParameters('flat');
        gait = GaitParameters(robot);
        controller = QuadrupedController(robot, gait, terrain);
        initialized = true;
    end

    bodyPosition = double(bodyPositionIn(:).');
    bodyVelocity = double(bodyVelocityIn(:).');
    bodyRPY = double(bodyRPYIn(:).');

    [jointAnglesOut, footWorldOut, landingPointsOut, phaseInfoOut, hipPointsOut] = ...
        controller.update(double(timeIn), bodyPosition, bodyVelocity, bodyRPY);

    jointAnglesOut = jointAnglesOut(:);
    footWorldOut = footWorldOut;
    landingPointsOut = landingPointsOut;
    phaseInfoOut = phaseInfoOut;
    hipPointsOut = hipPointsOut;
end
