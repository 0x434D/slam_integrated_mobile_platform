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
% Last Modified:   31.01.2022

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

% Create visual representation
td = figure;
set(td, 'Position', [100 100 1000 650])
sgtitle('Finding time delay using cross-correlation')
subplot(2,2,[1 2])
plot(norm_gnssInt,'LineWidth',1.5);
hold on
plot(norm_scanInt);
ylabel('Distance to start [m]')
xlabel("Time since start [" + num2str(stepSize) + "s]")
title('Un-matched trajectory times')
legend('GNSS','Scanner');

subplot(2,2,3)
[c,lags] = xcorr(norm_scanInt,norm_gnssInt);
stem(lags,c)
hold on
grid on
title('Cross-correlation')
xlabel("Delay [" + num2str(stepSize) + "s]")
ylabel('Magnitude')

subplot(2,2,4)
if timeOffset<=0
    plot(norm_gnssInt,'LineWidth',1.5);
    hold on
    plot(norm_scanInt(1+abs(timeOffset):end));
    title('Matched trajectory times')
    ylabel('Distance to start [m]')
    xlabel("Time since start [" + num2str(stepSize) + "s]")
elseif timeOffset
    plot(norm_gnssInt(1+abs(timeOffset):end),'LineWidth',1.5);
    hold on
    plot(norm_scanInt);
    title('Matched trajectory times')
    ylabel('Distance to start [m]')
    xlabel("Time since start [" + num2str(stepSize) + "s]")
end
legend('GNSS','Scanner');

% More plots
% figure;
% scatter(traj_gnssRed(:,1),traj_gnssRed(:,2));
% hold on
% scatter(traj_scanRed(:,1),traj_scanRed(:,2));
% legend('GNSS','Scanner');
% 
% figure;
% plot(traj_gnss(:,4)-traj_gnss(1,4),norm_gnss);
% hold on
% plot(traj_scan(:,1)-traj_scan(1,1),norm_scan);
% legend('GNSS','Scanner');
% 
% figure;
% [c,lags] = xcorr(norm_scanInt(end:-1:1),norm_gnssInt(end:-1:1));
% stem(lags,c)

% Return time offset
timeOffset = timeOffset * stepSize;
timeOffsetInv = timeOffsetInv * stepSize;
end
