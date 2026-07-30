classdef TerrainParameters

    properties

        %% ==========================================================
        % Terrain Type
        %% ==========================================================
        Type string = "Flat"
        % Flat
        % Inclined
        % Stair
        % Uneven

        %% ==========================================================
        % Inclination (deg)
        %% ==========================================================
        Pitch double = 0      % + : uphill
        Roll  double = 0      % + : left side up
        Yaw   double = 0

        %% ==========================================================
        % Surface Property
        %% ==========================================================
        Friction double = 0.8

        %% ==========================================================
        % Height Function
        %% ==========================================================
        HeightFunction

    end

    methods

        function obj = TerrainParameters(type)

            if nargin > 0
                obj.Type = string(type);
            end

            switch lower(obj.Type)

                case "flat"

                    obj.Pitch = 0;
                    obj.Roll  = 0;
                    obj.Yaw   = 0;

                case "inclined"

                    obj.Pitch = 24;
                    obj.Roll  = 0;
                    obj.Yaw   = 0;

                case "stair"

                    obj.Pitch = 0;

                case "uneven"

                    obj.Pitch = 0;

                otherwise

                    error("Unknown Terrain Type.");

            end

            obj.HeightFunction = @(x,y) ...
                obj.getHeight(x,y);

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function h = getHeight(obj,x,y)

            switch lower(obj.Type)

                case "flat"

                    h = zeros(size(x));

                case "inclined"

                    h = tand(obj.Pitch).*x;

                case "stair"

                    stepHeight = 30;
                    stepWidth  = 150;

                    h = floor(x/stepWidth)*stepHeight;

                case "uneven"

                    h = 15*sin(x/120).*cos(y/80);

                otherwise

                    h = zeros(size(x));

            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function n = getNormal(obj)

            pitch = deg2rad(obj.Pitch);
            roll  = deg2rad(obj.Roll);

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

            R = Ry*Rx;

            n = R*[0;0;1];

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function R = getTerrainRotation(obj)

            pitch = deg2rad(obj.Pitch);
            roll  = deg2rad(obj.Roll);
            yaw   = deg2rad(obj.Yaw);

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

        end

    end

end
