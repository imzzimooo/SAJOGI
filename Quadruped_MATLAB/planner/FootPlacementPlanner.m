classdef FootPlacementPlanner < handle

    properties

        robot
        gait
        terrain

    end

    methods

        %% ======================================================
        % Constructor
        %% ======================================================

        function obj = FootPlacementPlanner(robot,gait,terrain)

            obj.robot = robot;
            obj.gait = gait;
            obj.terrain = terrain;

        end

        %% ======================================================
        % Landing Point
        %% ======================================================

        function landing = getLandingPoint( ...
                obj,...
                bodyPos,...
                bodyVel,...
                leg)

            %--------------------------------------------
            % Hip Position
            %--------------------------------------------

            hip = obj.getHipLocation(leg);
            stanceBase = hip;

            %--------------------------------------------
            % Raibert Foot Placement
            %--------------------------------------------

            vx = bodyVel(1);

            x = stanceBase(1);

            x = x ...
                + obj.gait.StepLength/2 ...
                + 0.5*vx*obj.gait.T_stance;

            y = stanceBase(2);

            z = 0;

            %--------------------------------------------
            % Slope Compensation
            %--------------------------------------------

            pitch = deg2rad(obj.terrain.Pitch);

            x = x*cos(pitch);

            z = z + x*sin(pitch);

            %--------------------------------------------
            % Terrain Height
            %--------------------------------------------

            z = z + ...
                obj.terrain.getHeight(x,y);

            landing = bodyPos + [x y z];

        end

        %% ======================================================
        % Hip Position
        %% ======================================================

        function hip = getHipLocation(obj,leg)

            if isprop(obj.robot, 'HipJointPos') && ~isempty(obj.robot.HipJointPos)
                hip = obj.robot.HipJointPos(leg,:);
            else
                L = obj.robot.BodyLength;
                W = obj.robot.BodyWidth;

                switch leg

                    case 1      % LF
                        hip = [ L/2  -W/2  0 ];

                    case 2      % RF
                        hip = [ L/2   W/2  0 ];

                    case 3      % LR
                        hip = [ -L/2 -W/2  0 ];

                    case 4      % RR
                        hip = [ -L/2  W/2  0 ];

                end
            end

        end

        %% ======================================================
        % Adaptive Step Length
        %% ======================================================

        function L = getAdaptiveStepLength(obj)

            pitch = abs(obj.terrain.Pitch);

            gain = 0.01;

            L = obj.gait.StepLength * ...
                (1-gain*pitch);

            L = max(L,0.6*obj.gait.StepLength);

        end

        %% ======================================================
        % Adaptive Swing Height
        %% ======================================================

        function H = getAdaptiveSwingHeight(obj)

            pitch = abs(obj.terrain.Pitch);

            gain = 0.02;

            H = obj.gait.SwingHeight ...
                *(1+gain*pitch);

        end

    end

end