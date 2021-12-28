%% Georeferencing - Trajectory matching
% Author: SIMP Team
clearvars
close all
format longG
clc
fprintf('Georeferencing - Trajectory matching\n')

addpath('timeMatching')
step = 0.1;

%% Define Data
% Define lever arm [m]
leverArm = [-0.011 0.134 0.202]';       % from CAD model

% Load GNSS trajectory (format: [time [s], X [m], Y [m], Z [m]])
load('Data\testGNSS_RTK.mat','POS','POS_label','GPS2','GPS2_label');

% Select GPS2 (GNSS only) or KF-Solution (GNSS + IMU)
% GNSS = lla2ecef(POS(:,3:5));            % transform to ECEF [m]
% GNSS = [POS(:,2)*1e-6 GNSS];            % add time in [s]
% GNSS = lla2ecef(GPS2(:,8:10));          % transform to ECEF [m]
% GNSS = [GPS2(:,2)*1e-6 GNSS];           % add time in [s]
flatRef = [mean(GPS2(:,8:9)) 0];                % reference point for flat earth [°, °, m]
GNSS = lla2flat(GPS2(:,8:10),flatRef(1:2),0,0); % flat earth coordinates [m]
GNSS = [GNSS(:,2) GNSS(:,1) GNSS(:,3)];         % Switch order to X, Y, Z
GNSS = [GPS2(:,2)*1e-6 GNSS];                   % add time in [s]

% Load Scanner trajectory (format: [time [s], X [m], Y [m], Z [m] ...])
ScanRaw = load('Data\testSCAN_RTK.txt');

% Add lever arm to scan trajectory with orientation from scanner
additionalRot = [1 0 0; 0 1 0; 0 0 1];
Scan = simulateGNSS(ScanRaw, leverArm, additionalRot);
Scan = ScanRaw;
ScanBackup = Scan;                        % save original Scan trajectory

% Reduce scan trajectory times 
Scan(:,1) = Scan(:,1)-Scan(1,1);

% Delete GNSS measurements bevore moving via time matching
[timeOffset,~] = findTimeDelay(Scan, [GNSS(:,2:4) GNSS(:,1)], 0.01);
close all   
[~,idx] = min(abs(GNSS(:,1)-GNSS(1,1)-timeOffset)); % find index of time delay
GNSS = GNSS(idx:end,:);                             % now same trajectory start as Scan

% Save GNSS trajectory as variable
save('GNSS_Trajectory_test.mat','GNSS');

% Get mean time step size
ScanTSS = mean(diff(Scan(:,1)));
GNSSTSS = mean(diff(GNSS(:,1)));

% Match times
timeOffset = GNSS(1,1)-Scan(1,1);
Scan(:,1) = Scan(:,1)+timeOffset;

% % Reduce coordinates for numeric stability (relevant for ECEF)
% offset = mean(GNSS(:,2:4));
% GNSS(:,2:4) = GNSS(:,2:4)-offset;
% % Scan(:,2:4) = Scan(:,2:4)-offset;

%% Coarse trajectory match
[Scan,rotScale,translation] = coarseMatch(GNSS, Scan, timeOffset);

% Plot trajectories
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4),'b')
hold on
view([60 55])
grid on
plot3(Scan(:,2),Scan(:,3),Scan(:,4),'g')
legend('GNSS','SCAN','Location','NorthWest')
title('Coarse Trajectory Match Results')
view([90 90])
% print('-dpng','-r200',"CoarseTrafo.png")

%% Accurate trajectory match
[Scan,rotScale,translation] = accurateMatch(GNSS, Scan, timeOffset, rotScale, translation, 7, 200, 1, 0);

% % Add initial offset (relevant for ECEF)
% translation = translation + offset';
% GNSS(:,2:4) = GNSS(:,2:4) + offset;

%% Test transformation on original trajectory
% Transform trajectory and plot anew
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4),'b')
hold on 
view([60 55])
grid on
for i = 1:length(ScanBackup)
   ScanBackup(i,2:4) = rotScale * ScanBackup(i,2:4)' + translation;
end
plot3(ScanBackup(:,2),ScanBackup(:,3),ScanBackup(:,4),'g')
axis equal
legend('GNSS','SCAN','Location','NorthWest')
title('Combined Transformation (Coarse + Accurate)')
view([90 90])
% print('-dpng','-r200',"AccurateTrafo_Good.png")

%% Estimate accuracy
bound = 200;
for i = 1:length(GNSS)
    % Get approximate Scan index
    idx = round((GNSS(i,1)-timeOffset)/ScanTSS)+1;
    if idx > length(Scan)
        idx = length(Scan);
    end

    % Estimate lower and upper bound for threshold
    if idx-round(bound/2) < 1
        idxLB = 1;
    else
        idxLB = idx-round(bound/2);
    end

    if idx+round(bound/2) > length(Scan)
        idxUB = length(Scan);
    else
        idxUB = idx+round(bound/2);
    end

    % Find closest Scan point in threshold and save index and distance
    [~, matchIDX] = min(vecnorm(Scan(idxLB:idxUB,2:4)-GNSS(i,2:4),2,2));
    dist = norm(Scan(idxLB + matchIDX - 1,2:4)-GNSS(i,2:4));
    distxy = norm(Scan(idxLB + matchIDX - 1,2:3)-GNSS(i,2:3));
    distx = norm(Scan(idxLB + matchIDX - 1,2)-GNSS(i,2));
    disty = norm(Scan(idxLB + matchIDX - 1,3)-GNSS(i,3));
    distz = norm(Scan(idxLB + matchIDX - 1,4)-GNSS(i,4));
    % order: [GNSSIDX, ScanIDX, distance, distance in XY, distance in x,y,z]
    match(i,:) = [i, idxLB + matchIDX - 1, dist, distxy, distx, disty, distz];
end
stdev = norm(match(:,3))/sqrt(length(match));         % standard deviation 
stdevxy = norm(match(:,4))/sqrt(length(match));       % standard deviation x,y
meandiff = sum(match(:,3))/length(match);             % mean point match distance
meandiffxy = sum(match(:,4))/length(match);           % mean point match distance x,y
[stdx, stdy, stdz] = deal(norm(match(:,5))/sqrt(length(match)), norm(match(:,6))/sqrt(length(match)), norm(match(:,7))/sqrt(length(match)));
[meanx, meany, meanz] = deal(sum(match(:,5))/length(match), sum(match(:,6))/length(match), sum(match(:,7))/length(match));
fprintf('\n3D Accuracy (X, Y, Z):\n')
fprintf('Mean point match distance: %.3f m\n',meandiff)
fprintf('Standard deviation: %.3f m\n',stdev)
fprintf('\n2D Accuracy (X, Y):\n')
fprintf('Mean point match distance: %.3f m\n',meandiffxy)
fprintf('Standard deviation: %.3f m\n',stdevxy)
fprintf('\nAccuracy in X, Y, Z:\n')
fprintf('Mean point match distance (X,Y,Z): [%.3f %.3f %.3f] m\n',meanx,meany,meanz)
fprintf('Standard deviation (X,Y,Z): [%.3f %.3f %.3f] m\n',stdx,stdy,stdz)

% Plot point match distance in all axes
figure
hold on
grid on
plot(1:length(match), match(:,5));
plot(1:length(match), match(:,6));
plot(1:length(match), match(:,7));
legend('in X','in Y','in Z')
title('Point match distance')
xlabel('Time')
ylabel('Distance [m]')
% print('-dpng','-r200',"Error_withoutLever.png")

% TODO: 
%       - irgendwie winkelbild noch benutzen?
%       - Gewichtsmatrix --> Irgendwann später
%       - Scan mehr Punkte oder GNSS meh Punkte unabhängig
%       - Flat Earth Transformation statt ECEF wegen der Drehung --> So besser
%         matchbar --> Am Ende dann flat2lla und lla2ecef (etwas umständlich aber naja)
%       - Z-Koordinate ist Hauptproblem, wie erwartbar --> Vll runter gewichten
%         in P matrix?
%       - WIeso Z so gut?