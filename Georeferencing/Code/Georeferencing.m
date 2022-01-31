%% Georeferencing
% Author: SIMP Team
clearvars
close all
format longG
clc
fprintf('Georeferencing:\n')

addpath(genpath('..\'))

%% Load data and define parameters
fprintf('\nLoading Data\n')

% Define lever arm between scanner and GNSS antenna [m]
leverArm = [-0.011 0.134 0.397]';       % from CAD model
step = 0.1;

[GNSSFileName,GNSSPathName,~] = uigetfile('*.txt;*.mat', 'Select the input GNSS Trajectory File', '..\Data');
[ScanTFileName,ScanTPathName,~] = uigetfile('*.txt;*.mat', 'Select the input Scan Trajectory File', '..\Data');
[ScanPCFileName,ScanPCPathName,~] = uigetfile('*.laz;*.las', 'Select the input Scan Pointcloud File', '..\Data');
imgPath = uigetdir('..\Data','Select GoPro Image Folder');

% Load GNSS trajectory (format: [time [s], X [m], Y [m], Z [m]])
fprintf('\tLoading GNSS trajectory\n')
load([GNSSPathName GNSSFileName],'POS','POS_label','GPS_1','GPS_label');

% Select GPS1 (GNSS only) or KF-Solution (GNSS + IMU) and transform to flat
% earth frame for easier usage and orientation
flatRef = [mean(GPS_1(:,9:10)) 0];                  % reference point for flat earth [°, °, m]
GNSS = lla2flat(GPS_1(:,9:11),flatRef(1:2),0,0);    % flat earth coordinates [m]
GNSS = [GNSS(:,2) GNSS(:,1) GNSS(:,3)];             % Switch order to X, Y, Z

% Load Scanner trajectory (format: [time [s], X [m], Y [m], Z [m] ...])
fprintf('\tLoading Scan trajectory\n')
ScanRaw = load([ScanTPathName ScanTFileName]);

% Add lever arm to scan trajectory with orientation from scanner
Scan = simulateGNSS(ScanRaw, leverArm);
% Scan = ScanRaw;
ScanBackup = Scan;                    	% save original Scan trajectory

% Reduce scan trajectory times 
Scan(:,1) = Scan(:,1)-Scan(1,1);

% Load Scan Point Cloud Data
fprintf('\tLoading point cloud\n')
ScanPC = lasdata([ScanPCPathName, ScanPCFileName], 'loadall');

% Load image data
fprintf('\tLoad image data\n')
addpath(imgPath)
imgList = ls(imgPath);
imgList = imgList(3:end,:); % first two entries are not files

% Read images (and dimensions, times) in a loop and save in img structure
dsFactor = 3;                                           % downsample factor
iDim = [2880/dsFactor 5760/dsFactor 3 size(imgList,1)];	% size of img array
img = zeros(iDim(1),iDim(2),iDim(3),iDim(4),'uint8');  	% [x y rgb frame]
wait = waitbar(0,"Loading images (0/" + num2str(size(imgList,1)) + ")");
for i = 1:size(imgList,1)
    fullImg = imread(imgList(i,:));
    img(:,:,:,i) = fullImg(1:dsFactor:end,1:dsFactor:end,:);
    frameDates(i,:) = datetime(imfinfo(imgList(i,:)).DateTime,...
                               'InputFormat','yyyy:MM:dd HH:mm:ss');
    frameTimes(i,1) = datenum(frameDates(i,:))*24*3600;	% MATLAB time [s]
    waitbar(i/size(imgList,1),wait,"Loading images (" + num2str(i) + "/"+ num2str(size(imgList,1)) + ")");
end
close(wait);

%% Calculate Time Offset
fprintf('\nCalculating time offset\n')

% Get GPS time 
epoch = datetime(1980,1,6,'TimeZone','UTCLeapSeconds');
dtUTC = datenum(epoch + days(GPS_1(:,6)*7) + seconds(GPS_1(:,5)*1e-3) + hours(1));
GNSS = [dtUTC*24*3600 GNSS];    % add time in [seconds]

% Delete GNSS measurements before moving via time matching
[timeOffset,~] = findTimeDelay(Scan, [GNSS(:,2:4) GNSS(:,1)], 0.01);
[~,idx] = min(abs(GNSS(:,1)-GNSS(1,1)-timeOffset)); % find index of time delay
GNSS = GNSS(idx:end,:);                             % now same trajectory start as Scan

% Get mean time step size
ScanTSS = mean(diff(Scan(:,1)));
GNSSTSS = mean(diff(GNSS(:,1)));

% Match times
timeOffset = GNSS(1,1)-Scan(1,1);
Scan(:,1) = Scan(:,1)+timeOffset;

%% Coarse trajectory match
fprintf('\nCoarse trajectory match\n')
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

%% Accurate trajectory match
fprintf('\nAccurate trajectory match\n')
[Scan,rotScale,translation] = accurateMatch(GNSS, Scan, timeOffset, rotScale, translation, 7, 200, 1, 0);

%% Test transformation on original trajectory
fprintf('\nTest transformation on original trajectory\n')

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
% axis equal
legend('GNSS','SCAN','Location','NorthWest')
title('Combined Transformation (Coarse + Accurate)')
view([90 90])

%% Estimate point matching accuracy
fprintf('\nEstimate point matching accuracy\n')

% Find corresponding match points in a loop
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
    % match order: [GNSSIDX, ScanIDX, distance, dist in XY, dist in x,y,z]
    [~, matchIDX] = min(vecnorm(Scan(idxLB:idxUB,2:4)-GNSS(i,2:4),2,2));
    dist = norm(Scan(idxLB + matchIDX - 1,2:4)-GNSS(i,2:4));
    distxy = norm(Scan(idxLB + matchIDX - 1,2:3)-GNSS(i,2:3));
    distx = norm(Scan(idxLB + matchIDX - 1,2)-GNSS(i,2));
    disty = norm(Scan(idxLB + matchIDX - 1,3)-GNSS(i,3));
    distz = norm(Scan(idxLB + matchIDX - 1,4)-GNSS(i,4));
    match(i,:) = [i, idxLB + matchIDX - 1, dist, distxy, distx, disty, distz];
end

% Calculate standard deviations and mean point distances
stdev = norm(match(:,3))/sqrt(length(match));         % standard deviation 
stdevxy = norm(match(:,4))/sqrt(length(match));       % standard deviation x,y
meandiff = sum(match(:,3))/length(match);             % mean point match distance
meandiffxy = sum(match(:,4))/length(match);           % mean point match distance x,y
[stdx, stdy, stdz] = deal(norm(match(:,5))/sqrt(length(match)), norm(match(:,6))/sqrt(length(match)), norm(match(:,7))/sqrt(length(match)));
[meanx, meany, meanz] = deal(sum(match(:,5))/length(match), sum(match(:,6))/length(match), sum(match(:,7))/length(match));

% Output results
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

%% Transform Point Cloud
fprintf('\nTransform point cloud\n')
PC_transf = [ScanPC.x, ScanPC.y, ScanPC.z] * rotScale' + translation';

%% Colorize Point Cloud
fprintf('\nColorize point cloud\n')
RGB = colorCoding(PC_transf, ScanPC.gps_time, ScanBackup(1,1),...
                  timeOffset, GNSS, Scan, rotScale, iDim, img, frameTimes);
ScanPC.red = uint16(RGB(:,1));
ScanPC.green = uint16(RGB(:,2));
ScanPC.blue = uint16(RGB(:,3));

%% Remove Moving Objects
fprintf('\nRemove moving objects\n')

% parameters for 100% point cloud!
voxelLength = 0.5;
timeDiff = 5;

del = removeMovingObjects(PC_transf, ScanPC.gps_time, voxelLength, timeDiff);

ScanPC.intensity(del) = [];
ScanPC.bits(del) = [];
ScanPC.classification(del) = [];
ScanPC.user_data(del) = [];
ScanPC.scan_angle(del) = [];
ScanPC.point_source_id(del) = [];
ScanPC.gps_time(del) = [];
ScanPC.red(del) = [];
ScanPC.green(del) = [];
ScanPC.blue(del) = [];
ScanPC.selection(del) = [];

PC_transf(del,:) = [];

ScanPC.header.number_of_point_records = size(ScanPC.x,1);
ScanPC.header.number_of_points_by_return(1) = ScanPC.header.number_of_point_records;

%% Ground Classification
fprintf('\nGround Classification\n')
gridResolution = 1;
ElevationThreshold = 0.2;

% ScanPC.classification = int8(segmentGroundSMRF(pointCloud(PC_transf), gridResolution, 'ElevationThreshold', ElevationThreshold));

%% Save final cloud
fprintf('\nSave final cloud\n')

% Save finally in ECEF
PC_transf = lla2ecef(flat2lla(PC_transf(:,[2,1,3]), flatRef(1:2),0, 0));

ScanPC.header.max_x = max(PC_transf(:,1));
ScanPC.header.min_x = min(PC_transf(:,1));
ScanPC.header.max_y = max(PC_transf(:,2));
ScanPC.header.min_y = min(PC_transf(:,2));
ScanPC.header.max_z = max(PC_transf(:,3));
ScanPC.header.min_z = min(PC_transf(:,3));

ScanPC.header.x_offset = mean(PC_transf(:,1));
ScanPC.header.y_offset = mean(PC_transf(:,2));
ScanPC.header.z_offset = mean(PC_transf(:,3));

ScanPC.x = PC_transf(:,1);
ScanPC.y = PC_transf(:,2);
ScanPC.z = PC_transf(:,3);

% write_las(ScanPC, [ScanPCPathName, 'GeoreferencedPointcloud.las']);
