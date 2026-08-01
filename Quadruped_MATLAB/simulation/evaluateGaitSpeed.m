function maxOmega = evaluateGaitSpeed(desiredVelocity, swingTime)
% evaluateGaitSpeed Evaluate max joint angular velocity for a gait.
%   maxOmega = evaluateGaitSpeed(desiredVelocity, swingTime)

    rootDir = fileparts(fileparts(mfilename('fullpath')));
    if ~contains(path, rootDir)
        addpath(genpath(rootDir));
    end

    robot = RobotParameters();
    robot.DesiredVelocity = desiredVelocity;
    robot.SwingTime = swingTime;
    gait = GaitParameters(robot);
    controller = QuadrupedController(robot, gait, TerrainParameters('flat'));

    timeStep = 0.02;
    duration = gait.T_stride * 2;
    timeVec = 0:timeStep:duration;
    jointHistory = zeros(length(timeVec), 12);

    bodyPosition = [0 0 robot.BodyHeight];
    bodyVelocity = [desiredVelocity 0 0];
    bodyRPY = [0 0 0];

    for idx = 1:length(timeVec)
        t = timeVec(idx);
        [jointAngles, ~, ~, ~, ~] = controller.update(t, bodyPosition, bodyVelocity, bodyRPY);
        jointHistory(idx,:) = jointAngles';
        bodyPosition(1) = bodyPosition(1) + bodyVelocity(1) * timeStep;
    end

    dt = diff(timeVec(:));
    dq = diff(jointHistory, 1, 1) ./ dt;
    maxOmega = max(abs(dq), [], 'all');
end
