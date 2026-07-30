function PlotJointAngles(data)
% PlotJointAngles Plot joint-angle history from a simulation log.
%   data - either a struct with fields time and jointAngles,
%          or a path to a .mat file containing a 'log' struct.

    if nargin < 1 || isempty(data)
        if exist('debug_log.mat', 'file') == 2
            loaded = load('debug_log.mat', 'log');
            data = loaded.log;
        else
            error('PlotJointAngles requires a log struct or a debug_log.mat file.');
        end
    end

    if ischar(data) || isstring(data)
        loaded = load(char(data), 'log');
        data = loaded.log;
    end

    if ~isstruct(data)
        error('PlotJointAngles expects a struct or .mat file path.');
    end

    if ~isfield(data, 'time') || ~isfield(data, 'jointAngles')
        error('Input log must contain time and jointAngles fields.');
    end

    t = data.time(:);
    q = data.jointAngles;

    if size(q,1) ~= length(t)
        q = q';
    end

    nJoints = size(q,2);
    figure('Name', 'Joint Angles over Time');
    subplot(2,1,1);
    plot(t, q);
    xlabel('Time [s]');
    ylabel('Joint angle');
    title('Joint angle time history');
    grid on;

    jointNames = cell(1, nJoints);
    for i = 1:nJoints
        jointNames{i} = sprintf('Joint %d', i);
    end
    legend(jointNames, 'Location', 'bestoutside');

    subplot(2,1,2);
    if length(t) > 1
        dq = diff(q, 1, 1) ./ max(diff(t), eps);
        plot(t(2:end), dq);
        xlabel('Time [s]');
        ylabel('Angular velocity');
        title('Joint angular velocity over time');
        grid on;
        legend(jointNames, 'Location', 'bestoutside');
    else
        axis off;
        text(0.5, 0.5, 'Not enough samples for velocity plot.', 'HorizontalAlignment', 'center');
    end
end
