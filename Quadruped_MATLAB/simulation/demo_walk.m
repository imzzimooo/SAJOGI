%% demo_walk

clearvars;
clc;

rootDir = fileparts(fileparts(mfilename('fullpath')));
if ~contains(path, rootDir)
    addpath(genpath(rootDir));
end

fprintf('demo_walk: Starting standalone quadruped test.\n');

robot = RobotParameters();
terrain = TerrainParameters('flat');
gait = GaitParameters(robot);
controller = QuadrupedController(robot,gait,terrain);

bodyPosition = [0 0 robot.BodyHeight];
bodyVelocity = [robot.DesiredVelocity 0 0];
bodyRPY = [0 0 0];

timeStep = 0.02;
duration = gait.T_stride * 2;

realTimeRecurDyn = false; % true: send state each time step, false: send only final state

if realTimeRecurDyn
    RecurDynInterface.connect();
end

timeVec = 0:timeStep:duration;
nSteps = length(timeVec);
jointHistory = zeros(nSteps, 12);
footHistory = zeros(nSteps, 4, 3);
landingHistory = zeros(nSteps, 4, 3);
phaseHistory = zeros(nSteps, 4, 2);
hipHistory = zeros(nSteps, 4, 3);

for idx = 1:nSteps
    t = timeVec(idx);
    [jointAngles, footWorld, landingPoints, phaseInfo, hipPoints] = controller.update(t, bodyPosition, bodyVelocity, bodyRPY);
    jointHistory(idx,:) = jointAngles';
    footHistory(idx,:,:) = footWorld;
    landingHistory(idx,:,:) = landingPoints;
    phaseHistory(idx,:,:) = phaseInfo;
    hipHistory(idx,:,:) = hipPoints;

    if realTimeRecurDyn
        realtimeState.time = t;
        realtimeState.bodyPosition = bodyPosition;
        realtimeState.jointAngles = jointAngles;
        realtimeState.footWorld = footWorld;
        realtimeState.landingPoints = landingPoints;
        realtimeState.phaseInfo = phaseInfo;
        RecurDynInterface.sendState(realtimeState);
    end

    bodyPosition(1) = bodyPosition(1) + bodyVelocity(1) * timeStep;
end

log = struct();
log.time = timeVec(:);
log.footPositions = footHistory;
log.jointAngles = jointHistory;
log.landingPoints = landingHistory;
log.phase = phaseHistory;
log.hipPoints = hipHistory;
save('debug_log.mat', 'log');

fprintf('demo_walk: Completed %d steps.\n', nSteps);
PlotFoot(log);
PlotJointAngles(log);

state.time = timeVec(end);
state.bodyPosition = bodyPosition;
state.jointAngles = jointAngles;
state.log = log;

if ~realTimeRecurDyn
    RecurDynInterface.connect();
    RecurDynInterface.sendState(state);
    fprintf('demo_walk: RecurDyn interface called.\n');
end
