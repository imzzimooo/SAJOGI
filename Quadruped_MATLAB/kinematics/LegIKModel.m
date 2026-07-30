classdef LegIKModel < handle

    properties
        robot
    end

    methods

        function obj = LegIKModel(robot)
            obj.robot = robot;
        end

        function theta = solve(obj, footPos, ~)
            % solve Compute leg inverse kinematics
            %   footPos - desired foot position relative to hip [x y z]
            %   ~       - unused leg index placeholder
            %   theta   - [hipYaw; hipPitch; kneePitch]

            x = footPos(1);
            y = footPos(2);
            z = footPos(3);

            % Hip yaw / abduction for lateral offset.
            % With x = forward, y = lateral, z = up, this is the yaw angle
            % around the body z-axis. If your CAD defines the joint sign
            % opposite, set RobotParameters.HipYawSign = -1.
            yawSign = 1;
            if isprop(obj.robot, 'HipYawSign')
                yawSign = obj.robot.HipYawSign;
            end
            theta1 = yawSign * atan2(y, x);

            % Effective planar distance in the sagittal plane
            r = sqrt(x^2 + y^2);

            L1 = obj.robot.UpperLeg;
            L2 = obj.robot.LowerLeg;

            D = (r^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2);
            D = min(max(D, -1), 1);

            theta3 = atan2(-sqrt(max(0, 1 - D^2)), D);
            theta2 = atan2(z, r) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));

            theta = [theta1; theta2; theta3];

        end

    end

end