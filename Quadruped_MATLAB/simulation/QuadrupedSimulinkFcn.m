function [jointAnglesOut, footWorldOut, landingPointsOut, phaseInfoOut, hipPointsOut] = QuadrupedSimulinkFcn(timeIn, bodyPositionIn, bodyVelocityIn, bodyRPYIn)
% QuadrupedSimulinkFcn
% MATLAB Function block wrapper for the quadruped controller.
% This block can be used inside a Simulink model to drive the existing
% MATLAB controller implementation.

    coder.extrinsic('RobotParameters', 'TerrainParameters', 'GaitParameters', 'QuadrupedController', 'RecurDynInterface');

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

    state.time = double(timeIn);
    state.bodyPosition = bodyPosition;
    state.bodyVelocity = bodyVelocity;
    state.bodyRPY = bodyRPY;
    state.jointAngles = jointAnglesOut;
    state.footWorld = footWorldOut;
    state.landingPoints = landingPointsOut;
    state.phaseInfo = phaseInfoOut;
    state.hipPoints = hipPointsOut;

    try
        RecurDynInterface.connect();
        RecurDynInterface.sendState(state);
    catch ME
        % RecurDyn가 없거나 연결이 실패해도 시뮬레이션은 계속 진행되도록 무시합니다.
        warning(ME.identifier, 'QuadrupedSimulinkFcn: RecurDyn 전송 실패 - %s', ME.message);
    end
end
