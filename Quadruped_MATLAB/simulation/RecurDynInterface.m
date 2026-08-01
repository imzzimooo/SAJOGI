classdef RecurDynInterface
    methods(Static)
        function connected = connect(varargin)
            persistent tClient;
            persistent isConnected;

            if isempty(isConnected)
                isConnected = false;
            end
            if isempty(tClient)
                tClient = [];
            end

            if ~isempty(tClient) && isConnected
                connected = true;
                return;
            end

            if nargin >= 2
                serverIP = varargin{1};
                serverPort = varargin{2};
            else
                % RecurDyn이 실행 중인 PC의 IP와 설정한 Port 번호 입력!
                serverIP = '127.0.0.1';
                serverPort = 5000;
            end

            try
                tClient = tcpclient(serverIP, serverPort);
                isConnected = true;
                connected = true;
                fprintf('RecurDynInterface: Successfully connected to %s:%d\n', serverIP, serverPort);
            catch ME
                tClient = [];
                isConnected = false;
                connected = false;
                warning(ME.identifier, 'RecurDynInterface: 연결에 실패했습니다. 오프라인 모드로 전환합니다. %s', ME.message);
            end
        end

        function sendState(state)
            persistent tClient;
            persistent isConnected;

            if isempty(isConnected)
                isConnected = false;
            end
            if isempty(tClient)
                tClient = [];
            end

            if nargin < 1
                error('RecurDynInterface.sendState requires a state structure or numeric vector.');
            end

            if isstruct(state)
                timeValue = double(state.time);
                bodyPosition = [];
                bodyVelocity = [];
                bodyRPY = [];
                jointAngles = [];

                if isfield(state, 'bodyPosition') && ~isempty(state.bodyPosition)
                    bodyPosition = double(state.bodyPosition(:).');
                end
                if isfield(state, 'bodyVelocity') && ~isempty(state.bodyVelocity)
                    bodyVelocity = double(state.bodyVelocity(:).');
                end
                if isfield(state, 'bodyRPY') && ~isempty(state.bodyRPY)
                    bodyRPY = double(state.bodyRPY(:).');
                end
                if isfield(state, 'jointAngles') && ~isempty(state.jointAngles)
                    jointAngles = double(state.jointAngles(:).');
                end

                dataToSend = [timeValue, bodyPosition, bodyVelocity, bodyRPY, jointAngles];
            else
                dataToSend = double(state(:).');
            end

            if isConnected && ~isempty(tClient)
                write(tClient, dataToSend, 'double');
            else
                if isstruct(state)
                    fprintf('[Offline] Time: %.3f, Joints: %d array\n', timeValue, length(jointAngles));
                else
                    fprintf('[Offline] Sent %d values\n', length(dataToSend));
                end
            end
        end

        function state = receiveState()
            persistent tClient;
            persistent isConnected;
            state = struct();

            if isempty(isConnected)
                isConnected = false;
            end
            if isempty(tClient)
                tClient = [];
            end

            if isConnected && ~isempty(tClient)
                try
                    rawData = read(tClient, 12, 'double');
                    state.success = true;
                    state.actualJointAngles = rawData;
                catch ME
                    state.success = false;
                    state.message = ME.message;
                end
            else
                state.success = false;
                state.message = 'Not connected.';
            end
        end
    end
end