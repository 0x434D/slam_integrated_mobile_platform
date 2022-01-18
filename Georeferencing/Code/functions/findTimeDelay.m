%% Simulate GNSS Trajectory in trajner System with Lever Arm Offset and trajner Orientation


% ----------------------------------------------------------------------------
% Input:        traj_scan       -   Scanner Trajectory as numeric array
%               traj_gnss       -   GNSS Trajectory as numeric array
%               stepSize        -   Step Size for Interpolation in seconds 
% Output:       timeOffset      -   Time Offset between Scan and GNSS Trajectory
%               timeOffsetInv   -   Time Offset of Inverse Trajectories (end-to-end offset)
% ----------------------------------------------------------------------------
% Authors:     SIMP-Project Team
% ----------------------------------------------------------------------------
% Last Modified:   05.12.2021

function [timeOffset,timeOffsetInv] = findTimeDelay(traj_scan, traj_gnss, stepSize)

% Reduce coordinates by first point coordinates %center of gravity
traj_gnssRed = traj_gnss(:,1:3) - traj_gnss(1,1:3);  % mean(traj_gnss(:,1:3));
traj_scanRed = traj_scan(:,2:4) - traj_scan(1,2:4);  % mean(traj_scan(:,2:4));

% calculate norm for each position
norm_gnss = vecnorm(traj_gnssRed,2,2);
norm_scan = vecnorm(traj_scanRed,2,2);

% interpolate to have equal amount of steps per time unit
norm_gnssInt = interp1(traj_gnss(:,4),norm_gnss,traj_gnss(1,4):stepSize:traj_gnss(end,4))';
norm_scanInt = interp1(traj_scan(:,1),norm_scan,traj_scan(1,1):stepSize:traj_scan(end,1))';

% find delay both ways
timeOffset = finddelay(norm_scanInt,norm_gnssInt);
timeOffsetInv = finddelay(norm_scanInt(end:-1:1),norm_gnssInt(end:-1:1));

% scale 
figure;
scatter(traj_gnssRed(:,1),traj_gnssRed(:,2));
hold on
scatter(traj_scanRed(:,1),traj_scanRed(:,2));
legend('GNSS','Scanner');

figure;
plot(traj_gnss(:,4)-traj_gnss(1,4),norm_gnss);
hold on
plot(traj_scan(:,1)-traj_scan(1,1),norm_scan);
legend('GNSS','Scanner');

figure;
plot(norm_gnssInt);
hold on
plot(norm_scanInt);
legend('GNSS','Scanner');

figure;
[c,lags] = xcorr(norm_scanInt,norm_gnssInt);
stem(lags,c)

figure;
[c,lags] = xcorr(norm_scanInt(end:-1:1),norm_gnssInt(end:-1:1));
stem(lags,c)

figure;
if timeOffset<=0
    plot(norm_gnssInt);
    hold on
    plot(norm_scanInt(1+abs(timeOffset):end));
elseif timeOffset
    plot(norm_gnssInt(1+abs(timeOffset):end));
    hold on
    plot(norm_scanInt);
end
legend('GNSS','Scanner');

timeOffset = timeOffset * stepSize;
timeOffsetInv = timeOffsetInv * stepSize;

end
