classdef BodyIKModel < handle

    properties

        robot
        terrain
    end

    methods

        %% ==========================================================
        % Constructor
        %% ==========================================================
        function obj = BodyIKModel(robot,terrain)

            obj.robot = robot;
            obj.terrain = terrain;

        end

        %% ==========================================================
        % Hip Position
        %% ==========================================================
        function hip = getHipPosition(obj)

            L = obj.robot.BodyLength;
            W = obj.robot.BodyWidth;

            hip = [

                 L/2  -W/2   0      % LF

                 L/2   W/2   0      % RF

                -L/2  -W/2   0      % LR

                -L/2   W/2   0      % RR

                ];

        end

        %% ==========================================================
        % World → Body
        %% ==========================================================
        function pBody = worldToBody(obj,pWorld,bodyPos,bodyRPY)

            roll  = deg2rad(bodyRPY(1));
            pitch = deg2rad(bodyRPY(2));
            yaw   = deg2rad(bodyRPY(3));

            Rx = [

                1 0 0

                0 cos(roll) -sin(roll)

                0 sin(roll) cos(roll)

                ];

            Ry = [

                cos(pitch) 0 sin(pitch)

                0 1 0

                -sin(pitch) 0 cos(pitch)

                ];

            Rz = [

                cos(yaw) -sin(yaw) 0

                sin(yaw) cos(yaw) 0

                0 0 1

                ];

            R = Rz*Ry*Rx;

            pBody = (R')*(pWorld-bodyPos)';

            pBody = pBody';

        end

        %% ==========================================================
        % Hip → Foot
        %% ==========================================================
        function htf = getHipToFoot(...
                obj,...
                footWorld,...
                bodyPos,...
                bodyRPY)

            hip = obj.getHipPosition();

            htf = zeros(4,3);

            for i=1:4

                footBody = obj.worldToBody(...
                    footWorld(i,:),...
                    bodyPos,...
                    bodyRPY);

                htf(i,:) = footBody - hip(i,:);

            end

        end

    end

end