classdef RobotParameters

    properties

        %% ==============================
        % Body Geometry
        % ===============================
        BodyLength         % mm
        BodyWidth          % mm
        BodyHeight         % Nominal body height (mm)

        %% ==============================
        % Leg Geometry
        % ===============================
        UpperLeg           % Hip -> Knee (mm)
        LowerLeg           % Knee -> Foot (mm)

        HipOffsetX         % Hip servo offset X (mm)
        HipOffsetY         % Hip servo offset Y (mm)

        %% ==============================
        % Gait Parameters
        % ===============================
        StepLength         % mm

        SwingHeight        % mm

        DesiredVelocity    % mm/s

        SwingTime          % sec

        PhaseLag

        PenetrationAngle

        %% ==============================
        % Initial Foot Position
        % ===============================
        FootY

        XShift

    end

    methods

        function obj = RobotParameters()

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % 반드시 자신의 CAD 치수로 수정
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% Body

            obj.BodyLength = 220;

            obj.BodyWidth = 120;

            obj.BodyHeight = 180;

            %% Leg

            obj.UpperLeg = 95;

            obj.LowerLeg = 110;

            obj.HipOffsetX = 38;

            obj.HipOffsetY = 32;

            %% Walking

            obj.StepLength = 70;

            obj.SwingHeight = 25;

            obj.DesiredVelocity = 300;

            obj.SwingTime = 0.25;

            obj.PhaseLag = [

                0

                0.5

                0.5

                0

                ];

            obj.PenetrationAngle = 15;

            %% Foot Position

            obj.FootY = 55;

            obj.XShift = 0;

        end

    end

end