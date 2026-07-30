classdef GaitParameters

    properties

        %% Timing
        T_swing            % Swing Time [s]
        T_stance           % Stance Time [s]
        T_stride           % Total Stride Time [s]

        %% Walking
        StepLength         % Step Length [mm]
        DesiredVelocity    % Desired Velocity [mm/s]

        %% Phase
        PhaseLag           % Phase offset of each leg

        %% Foot Trajectory
        SwingHeight        % Foot Clearance [mm]
        PenetrationAngle   % Penetration Angle [deg]

        %% Initial Foot Position
        BaseHeight
        FootY
        XShift

    end

    methods

        function obj = GaitParameters(robot)

            %----------------------------------------------------------
            % robot : RobotParameters class
            %----------------------------------------------------------

            obj.StepLength       = robot.StepLength;

            obj.DesiredVelocity  = robot.DesiredVelocity;

            obj.T_swing          = robot.SwingTime;

            %---------------------------------------------
            % Stance Time
            %
            % distance = velocity × time
            %
            % StepLength = v*Tstance/2
            %
            %---------------------------------------------

            obj.T_stance = ...
                (2*obj.StepLength)/obj.DesiredVelocity;

            obj.T_stride = ...
                obj.T_stance + obj.T_swing;

            obj.PhaseLag = robot.PhaseLag;

            obj.SwingHeight = robot.SwingHeight;

            obj.PenetrationAngle = ...
                robot.PenetrationAngle;

            obj.BaseHeight = robot.BodyHeight;

            obj.FootY = robot.FootY;

            obj.XShift = robot.XShift;

        end

    end

end