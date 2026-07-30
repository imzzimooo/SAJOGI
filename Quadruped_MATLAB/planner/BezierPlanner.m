classdef BezierPlanner < handle

    properties

        robot
        gait
        terrain

        controlPoints
        swingControlPoints
        stanceControlPoints

    end

    methods

        %% ==========================================================
        % Constructor
        %% ==========================================================

        function obj = BezierPlanner(robot,gait,terrain)

            obj.robot = robot;
            obj.gait = gait;
            obj.terrain = terrain;

            obj.controlPoints = obj.generateControlPoints();
            obj.swingControlPoints = obj.generateSwingControlPoints();
            obj.stanceControlPoints = obj.generateStanceControlPoints();

        end

        %% ==========================================================
        % Generate Control Points
        %% ==========================================================

        function cp = generateControlPoints(obj)

            cp = obj.generateSwingControlPoints();

        end

        %% ==========================================================
        % Generate Swing Control Points
        %% ==========================================================

        function cp = generateSwingControlPoints(obj)

            L = obj.gait.StepLength;
            H = obj.gait.SwingHeight;

            % A quintic Bézier with zero velocity at both ends.
            % This suppresses the sharp derivative jump at the
            % swing-to-stance transition.
            cp = [
                -L/2   0    0
                -L/2   0    0
                -L/3   0    H*0.60
                 0     0    H
                 L/3   0    H*0.60
                 L/2   0    0
                ];

        end

        %% ==========================================================
        % Generate Stance Control Points
        %% ==========================================================

        function cp = generateStanceControlPoints(obj)

            L = obj.gait.StepLength;

            % A cubic Bézier with zero velocity at both ends.
            % This makes the stance phase smoother than a pure linear ramp.
            cp = [
                 L/2   0    0
                 L/2   0    0
                -L/2   0    0
                -L/2   0    0
                ];

        end

        %% ==========================================================
        % Gait Phase
        %% ==========================================================

        function [mode,phase] = getPhase(obj,time,leg)

            lag = obj.gait.PhaseLag(leg);

            t = mod(time - lag*obj.gait.T_stride, obj.gait.T_stride);

            if t <= obj.gait.T_stance
                mode = 0;
                phase = t / obj.gait.T_stance;
            else
                mode = 1;
                phase = (t - obj.gait.T_stance) / obj.gait.T_swing;
            end

        end

        %% ==========================================================
        % Foot Position
        %% ==========================================================

        function pWorld = getFootPosition(obj,time,leg,landing)

            if nargin < 4 || isempty(landing)
                landing = [0 0 0];
            end

            [mode,phase] = obj.getPhase(time,leg);
            blend = 0.05;

            if mode == 0
                if phase > 1-blend
                    t = (phase - (1-blend)) / blend;
                    pA = obj.stanceTrajectory(phase);
                    pB = obj.swingTrajectory(0);
                    pTerrain = obj.blendTransition(pA, pB, t);
                else
                    pTerrain = obj.stanceTrajectory(phase);
                end
            else
                if phase < blend
                    t = phase / blend;
                    pA = obj.stanceTrajectory(1);
                    pB = obj.swingTrajectory(phase);
                    pTerrain = obj.blendTransition(pA, pB, t);
                else
                    pTerrain = obj.swingTrajectory(phase);
                end
            end

            R = obj.terrain.getTerrainRotation();
            offset = R * [obj.gait.StepLength/2; 0; 0];

            pWorld = (R * pTerrain')' - offset' + landing;
            pWorld(3) = pWorld(3) + obj.terrain.getHeight(pWorld(1), pWorld(2));

        end

        %% ==========================================================
        % Swing
        %% ==========================================================

        function p = swingTrajectory(obj,s)

            p = obj.evaluateBezier(obj.swingControlPoints,s);

        end

        %% ==========================================================
        % Stance
        %% ==========================================================

        function p = stanceTrajectory(obj,s)

            p = obj.evaluateBezier(obj.stanceControlPoints,s);

        end

        %% ==========================================================
        % Evaluate Bézier Curve
        %% ==========================================================

        function p = evaluateBezier(~,cp,s)

            n = size(cp,1) - 1;
            p = zeros(1,3);

            for i = 0:n
                B = nchoosek(n,i) * (1-s)^(n-i) * s^i;
                p = p + B * cp(i+1,:);
            end

        end

        %% ==========================================================
        % Blend Transition with Smooth Weight
        %% ==========================================================

        function p = blendTransition(~,pA,pB,t)

            t = min(max(t,0),1);
            w = t^3 * (10 - 15*t + 6*t^2);
            p = (1-w) * pA + w * pB;

        end

    end

end