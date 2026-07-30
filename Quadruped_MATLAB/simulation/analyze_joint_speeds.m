cd(fileparts(mfilename('fullpath')));
load('debug_log.mat','log');

t = log.time(:);
q = log.jointAngles;
if size(q,1) ~= length(t)
    q = q';
end

dq = diff(q,1,1) ./ max(diff(t), eps);

fprintf('samples=%d\n', size(q,1));
fprintf('max_abs_dq_each_joint=');
disp(max(abs(dq),[],1));
fprintf('max_abs_dq_overall=%.4f\n', max(abs(dq(:))));
fprintf('max_abs_dq_joint1=%.4f\n', max(abs(dq(:,1))));
fprintf('max_abs_dq_joint4=%.4f\n', max(abs(dq(:,4))));
