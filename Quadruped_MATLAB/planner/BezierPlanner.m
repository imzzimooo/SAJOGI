classdef BezierPlanner < handle

    properties

        robot
        gait
        terrain

        controlPoints

    end

    methods

        %% ==========================================================
        % Constructor
        %% ==========================================================

        function obj = BezierPlanner(robot,gait,terrain)

            obj.robot = robot;
            obj.gait = gait;
            obj.terrain = terrain;

            obj.controlPoints = ...
                obj.generateControlPoints();

        end

        %% ==========================================================
        % Generate Control Points
        %% ==========================================================

        function cp = generateControlPoints(obj)

            L = obj.gait.StepLength;

            H = obj.gait.SwingHeight;

            cp = [

                -L/2   0    0

                -L/2   0    0

                -L/2   0    H*0.30

                -L/3   0    H*0.60

                -L/6   0    H

                 0     0    H

                 L/6   0    H

                 L/3   0    H*0.60

                 L/2   0    H*0.30

                 L/2   0    0

                 L/2   0    0

                 L/2   0    0

                ];

        end

        %% ==========================================================
        % Gait Phase
        %% ==========================================================

        function [mode,phase] = getPhase(obj,time,leg)

            lag = obj.gait.PhaseLag(leg);

            t = mod( ...
                time-lag*obj.gait.T_stride,...
                obj.gait.T_stride);

            if t <= obj.gait.T_stance

                mode = 0;

                phase = t/obj.gait.T_stance;

            else

                mode = 1;

                phase = ...
                    (t-obj.gait.T_stance) ...
                    /obj.gait.T_swing;

            end

        end

        %% ==========================================================
        % Foot Position
        %% ==========================================================

        function pWorld = getFootPosition(obj,time,leg)

            [mode,phase] = ...
                obj.getPhase(time,leg);

            if mode==0

                pTerrain = ...
                    obj.stanceTrajectory(phase);

            else

                pTerrain = ...
                    obj.swingTrajectory(phase);

            end

            R = obj.terrain.getTerrainRotation();

            pWorld = (R*pTerrain')';

            pWorld(3)=pWorld(3)+...
                obj.terrain.getHeight( ...
                pWorld(1),...
                pWorld(2));

        end

        %% ==========================================================
        % Swing
        %% ==========================================================

        function p = swingTrajectory(obj,s)

            cp = obj.controlPoints;

            n=size(cp,1)-1;

            p=zeros(1,3);

            for i=0:n

                B=nchoosek(n,i) ...
                    *(1-s)^(n-i) ...
                    *s^i;

                p=p+B*cp(i+1,:);

            end

        end

        %% ==========================================================
        % Stance
        %% ==========================================================

        function p = stanceTrajectory(obj,s)

            L=obj.gait.StepLength;

            x=L/2-L*s;

            p=[x 0 0];

        end

    end

end