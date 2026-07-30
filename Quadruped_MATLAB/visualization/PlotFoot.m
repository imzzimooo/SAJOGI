function PlotFoot(data)
% PlotFoot Plot foot trajectories, landing points, and phase state.
%   data - either a struct with fields footPositions/landingPoints/phase,
%          or an Nx3 matrix of foot coordinates.

    if nargin < 1
        error('PlotFoot requires input data.');
    end

    if isstruct(data)
        footHistory = data.footPositions;
        landingHistory = data.landingPoints;
        phaseHistory = data.phase;
    else
        footHistory = data;
        landingHistory = [];
        phaseHistory = [];
    end

    if isempty(footHistory)
        error('PlotFoot received empty foot data.');
    end

    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Foot Trajectory and Landing Points');

    nLegs = size(footHistory,2);
    colors = lines(nLegs);

    for leg = 1:nLegs
        traj = squeeze(footHistory(:,leg,:));
        plot3(traj(:,1), traj(:,2), traj(:,3), '-o', 'Color', colors(leg,:), 'LineWidth', 1.5);

        if ~isempty(landingHistory)
            landingFinal = squeeze(landingHistory(end,leg,:));
            plot3(landingFinal(1), landingFinal(2), landingFinal(3), 'x', 'Color', colors(leg,:), 'MarkerSize', 8);
        end

        if ~isempty(phaseHistory)
            phaseFinal = phaseHistory(end,leg,:);
            text(traj(end,1), traj(end,2), traj(end,3), sprintf('Leg %d, mode=%d, phase=%.2f', leg, phaseFinal(1), phaseFinal(2)), 'Color', colors(leg,:), 'FontSize', 8);
        end
    end

    hold off;
end
