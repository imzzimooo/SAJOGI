classdef RecurDynInterface
    methods(Static)
        function connect(varargin)
            persistent tClient;
            persistent isConnected;

            if isempty(isConnected)
                isConnected = false;
            end
            if isempty(tClient)
                tClient = [];
            end

            if isempty(tClient)
                if nargin >= 2
                    serverIP = varargin{1};
                    serverPort = varargin{2};
                else
                    % RecurDyn이 실행 중인 PC의 IP와 설정한 Port 번호 입력
                    serverIP = '127.0.0.1';
                    serverPort = 5000;
                end

                try
                    tClient = tcpclient(serverIP, serverPort);
                    isConnected = true;
                    fprintf('RecurDynInterface: Successfully connected to %s:%d\n', serverIP, serverPort);
                catch ME
                    tClient = [];
                    isConnected = false;
                    warning(ME.identifier, 'RecurDynInterface: 연결에 실패했습니다. 오프라인 모드로 전환합니다. %s', ME.message);
                end
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
                error('RecurDynInterface.sendState requires a state structure.');
            end

            dataToSend = [state.time, state.jointAngles(:)']; 

            if isConnected && ~isempty(tClient)
                write(tClient, dataToSend, 'double');
            else
                fprintf('[Offline] Time: %.3f, Joints: %d array\n', state.time, length(state.jointAngles));
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
                rawData = read(tClient, 12, 'double');
                state.success = true;
                state.actualJointAngles = rawData;
            else
                state.success = false;
                state.message = 'Not connected.';
            end
        end
    end
end
