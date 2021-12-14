function [traj_matched] = matchTrajByTime(traj_scan, traj_gnss, timeDelay)



timeDelayTotal = traj_scan(1,1) - traj_gnss(1,4) - timeDelay;

traj_matched = zeros(size(traj_scan,1),size(traj_gnss,2));

for i=1:length(traj_scan)
    timeDiff = traj_scan(i,1) - timeDelayTotal - traj_gnss(:,4);
    traj_matched(i,:) = traj_gnss(abs(timeDiff) == min(abs(timeDiff)),:);
end


end