classdef QuadrupedController < handle

    properties

        robot
        gait
        terrain

        footPlanner
        bezierPlanner

        bodyIK
        legIK

    end

    methods

        %% =========================================================
        % Constructor
        %% =========================================================

        function obj = QuadrupedController(robot,gait,terrain)

            obj.robot = robot;
            obj.gait = gait;
            obj.terrain = terrain;

            obj.footPlanner = FootPlacementPlanner(robot,gait,terrain);
            obj.bezierPlanner = BezierPlanner(robot,gait,terrain);
            obj.bodyIK = BodyIKModel(robot,terrain);
            obj.legIK = LegIKModel(robot);

        end

        %% =========================================================
        % Update
        %% =========================================================

        function [jointAngles,footWorld,landingPoints,phaseInfo,hipPoints] = update(obj,time,bodyPosition,bodyVelocity,bodyRPY)

            footWorld = zeros(4,3);
            landingPoints = zeros(4,3);
            phaseInfo = zeros(4,2);
            hipPoints = zeros(4,3);

            for leg = 1:4
                hipPoints(leg,:) = obj.footPlanner.getHipLocation(leg);

                landing = obj.footPlanner.getLandingPoint(bodyPosition, bodyVelocity, leg);
                landingPoints(leg,:) = landing;

                [mode,phase] = obj.bezierPlanner.getPhase(time, leg);
                phaseInfo(leg,:) = [mode phase];

                footWorld(leg,:) = obj.bezierPlanner.getFootPosition(time, leg, landing);
            end

            hipToFoot = obj.bodyIK.getHipToFoot(footWorld, bodyPosition, bodyRPY);

            jointAngles = zeros(12,1);
            for leg = 1:4
                theta = obj.legIK.solve(hipToFoot(leg,:), leg);
                idx = (leg-1)*3 + 1;
                jointAngles(idx:idx+2) = theta;
            end

        end

    end

end