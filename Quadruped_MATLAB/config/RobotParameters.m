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
        HipYawSign         % Sign convention for hip yaw rotation

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

        %% ==============================
        % Home Foot Position (Body Frame)
        % ===============================
        HomeFootPos

        %% ==============================
        % Hip Joint Position (Body Frame)
        % ===============================
        HipJointPos

    end

    methods

        function obj = RobotParameters()

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % CAD 치수로 수정
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% Body

            obj.BodyLength = 302;

            obj.BodyWidth = 124;

            obj.BodyHeight = 149.7;

            %% Leg

            obj.UpperLeg = 154.03;

            obj.LowerLeg = 153.28;

            obj.HipOffsetX = 79.00;

            obj.HipOffsetY = 31.05;

            obj.HipYawSign = 1;

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

            obj.HipJointPos = [
                obj.BodyLength/2 - obj.HipOffsetX, -obj.BodyWidth/2 + obj.HipOffsetY, 0;   % LF
                obj.BodyLength/2 - obj.HipOffsetX,  obj.BodyWidth/2 - obj.HipOffsetY, 0;   % RF
               -obj.BodyLength/2 + obj.HipOffsetX, -obj.BodyWidth/2 + obj.HipOffsetY, 0;   % LB
               -obj.BodyLength/2 + obj.HipOffsetX,  obj.BodyWidth/2 - obj.HipOffsetY, 0;   % RB
            ];

            obj.HomeFootPos = obj.HipJointPos + [0, -obj.FootY, 0; 0, obj.FootY, 0; 0, -obj.FootY, 0; 0, obj.FootY, 0];

        end

    end

end