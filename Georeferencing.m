%% Georeferencing - Lever Arm Estimation
% Author: Johannes Ernst, Tim Kayser
clearvars
close all
format longG
clc
fprintf('Georeferencing - Lever Arm Estimation\n')

step = 1;

%% Load Data
% Create GNSS trajectory
GNSS = load('trajectory_odometry_slam_two_loops').trajectory;
Zcoord1 = linspace(0,0.2,length(GNSS)/4)';
Zcoord2 = linspace(0.2,0,length(GNSS)/4)';
Zcoord3 = linspace(0,0.2,length(GNSS)/4)';
Zcoord4 = linspace(0.2,0,length(GNSS)/4)';
GNSS = [GNSS(:,1), GNSS(:,2), [Zcoord1;Zcoord2;Zcoord3;Zcoord4;0;0;0]];
GNSS = [GNSS, linspace(60,360,length(GNSS))'];   % create time stamps

% Create Scanner trajectory
params = [1.2; 1.3; 1; 0; 0; pi/4; 10; 10; 0.6]; 	% simulate rotation and offset
Scan = Trafo9(GNSS(:,1:3),params);
Scan = Scan(1:3:end,:);                             % subsample Scan trajectory
Scan(:,1) = Scan(:,1)+rand(length(Scan),1)*0.02;    % create noise
Scan(:,2) = Scan(:,2)+rand(length(Scan),1)*0.02;
Scan(:,3) = Scan(:,3)+rand(length(Scan),1)*0.002;
% Scan(70:120,1) = Scan(70:120,1)+linspace(0,-0.3,51)';    % create more noise
% Scan(400:460,1) = Scan(400:460,1)+linspace(0,0.3,61)';
% Scan(220:270,1) = Scan(220:270,1)+linspace(0,-0.3,51)';
% Scan(100:170,2) = Scan(100:170,2)+linspace(0,-0.3,71)';
% Scan(450:550,2) = Scan(450:550,2)+linspace(0,0.15,101)';
% Scan(200:220,2) = Scan(200:220,2)+linspace(0,0.4,21)';
Scan = [Scan, linspace(0,300,length(Scan))'];       % create time stamps
ScanBackup = Scan;

% % Second trajectories
% test = load('trajektories.mat');
% GNSS = test.Xgt;
% Zcoord1 = linspace(0,0.2,length(GNSS)/4)';
% Zcoord2 = linspace(0.2,0,length(GNSS)/4)';
% Zcoord3 = linspace(0,0.2,length(GNSS)/4)';
% Zcoord4 = linspace(0.2,0,length(GNSS)/4)';
% % GNSS = [GNSS(:,1), GNSS(:,2), [Zcoord1;Zcoord2;Zcoord3;Zcoord4;0;0]];
% GNSS = [GNSS(:,1), GNSS(:,2), zeros(length(GNSS),1)];
% GNSS = [GNSS, linspace(60,360,length(GNSS))'];   % create time stamps
% 
% Scan = test.Xsl;
% Zcoord1 = linspace(0,0.2,length(Scan)/4)';
% Zcoord2 = linspace(0.2,0,length(Scan)/4)';
% Zcoord3 = linspace(0,0.2,length(Scan)/4)';
% Zcoord4 = linspace(0.2,0,length(Scan)/4)';
% % Scan = [Scan(:,1), Scan(:,2), [Zcoord1;Zcoord2;Zcoord3;Zcoord4;0;0;0]];
% Scan = [Scan(:,1), Scan(:,2), zeros(length(Scan),1)];
% Scan = [Scan, linspace(0,300,length(Scan))'];   % create time stamps
% ScanBackup = Scan;

% Get time step size
ScanTSS = Scan(2,4)-Scan(1,4);
GNSSTSS = GNSS(2,4)-GNSS(1,4);

% Plot trajectories
figure
plot3(GNSS(:,1),GNSS(:,2),GNSS(:,3))
hold on
grid on
plot3(Scan(:,1),Scan(:,2),Scan(:,3),'k')


%% Coarse trajectory match
% Match times
timeOffset = GNSS(1,4)-Scan(1,4);
Scan(:,4) = Scan(:,4)+timeOffset;

% Find initial offset and add to scan trajectory
initOffset = GNSS(1,1:3)-Scan(1,1:3);
Scan(:,1:3) = Scan(:,1:3)+initOffset;

% Plot trajectories
pause(step)
hold off
plot3(GNSS(:,1),GNSS(:,2),GNSS(:,3))
hold on
grid on
plot3(Scan(:,1),Scan(:,2),Scan(:,3),'k')

% Estimate furthest point from start in GNSS trajectory and plot
pause(step)
% [~,idxGNSS] = max(sqrt((GNSS(:,1)-GNSS(1,1)).^2+(GNSS(:,2)-GNSS(1,2)).^2+(GNSS(:,3)-GNSS(1,3)).^2))
[~,idxGNSS] = max(vecnorm((GNSS(:,1:3)-GNSS(1,1:3)),2,2));
scatter3(GNSS(idxGNSS,1),GNSS(idxGNSS,2),GNSS(idxGNSS,3),'r','filled');

% Get furthest point from start in Scan trajectory and plot
pause(step)
% [~,idxScan] = max(vecnorm((Scan(:,1:3)-Scan(1,1:3)),2,2));
idxScan = round((GNSS(idxGNSS,4)-timeOffset)/ScanTSS);
scatter3(Scan(idxScan,1),Scan(idxScan,2),Scan(idxScan,3),'r','filled');

% Estimate azimuth of both trajectories with first and furthest point
aziGNSS = mod(atan2(GNSS(idxGNSS,2)-GNSS(1,2),GNSS(idxGNSS,1)-GNSS(1,1)),2*pi);
aziScan = mod(atan2(Scan(idxScan,2)-Scan(1,2),Scan(idxScan,1)-Scan(1,1)),2*pi);

% Estimate rough z-axis rotation between trajectories
zRot = aziScan-aziGNSS;

% Rotate scanner trajectory
trafoCoarse = [1 1 1 0 0 -zRot 0 0 0]';
Scan(:,1:3) = Trafo9(Scan(:,1:3),trafoCoarse);

% Plot trajectories
pause(step)
hold off
plot3(GNSS(:,1),GNSS(:,2),GNSS(:,3))
hold on
grid on
plot3(Scan(:,1),Scan(:,2),Scan(:,3),'k')
scatter3(GNSS(idxGNSS,1),GNSS(idxGNSS,2),GNSS(idxGNSS,3),'r','filled');
scatter3(Scan(idxScan,1),Scan(idxScan,2),Scan(idxScan,3),'r','filled');

%% Accurate trajectory match
% Combined transformation (coarse and accurate)
rotScale = [trafoCoarse(1) 0 0; 0 trafoCoarse(2) 0; 0 0 trafoCoarse(3)]*...
           [cos(trafoCoarse(6)) -sin(trafoCoarse(6)) 0; sin(trafoCoarse(6)) cos(trafoCoarse(6)) 0; 0 0 1]*...
           [cos(trafoCoarse(5)) 0 sin(trafoCoarse(5)); 0 1 0; -sin(trafoCoarse(5)) 0 cos(trafoCoarse(5))]*...
           [1 0 0; 0 cos(trafoCoarse(4)) -sin(trafoCoarse(4)); 0 sin(trafoCoarse(4)) cos(trafoCoarse(4))];
translation = [0 0 0]' + rotScale*initOffset';

% Find iterative closest points
for t = 1:7 
    % Find closest point in GNSS trajectory for each point in scan trajectory
    for i = 1:length(Scan)
        
        % Get approximate GNSS index
        idx = round((Scan(i,4)-timeOffset)/GNSSTSS)+1;

        % Estimate lower and upper bound for threshold
        if idx-5 < 1
            idxLB = 1;
        else
            idxLB = idx-5;
        end

        if idx+5 > length(GNSS)
            idxUB = length(GNSS);
        else
            idxUB = idx+5;
        end

        % Find closest GNSS point in threshold and save index and distance
        [~, matchIDX] = min(vecnorm(GNSS(idxLB:idxUB,1:3)-Scan(i,1:3),2,2));
        dist = norm(GNSS(idxLB + matchIDX - 1,1:3)-Scan(i));
        match(i,:) = [i, idxLB + matchIDX - 1, dist];   % order: [ScanIDX, GNSSIDX, distance]
    end
    
    % Delete 5% of the worst matches (biggest difference)
    [~, sortIDX] = sort(match(:,3));        % sort by distance
    delElem = round(0.05*length(match));    % determine number of elements to be deleted
    sortIDX = sortIDX(1:end-delElem);       % delete elements
    scanIDX = sort(sortIDX);                % remaining scan indices

    % Estimate transformation
    trafoParam = Est9Trafo3D(Scan(match(scanIDX,1),1:3),GNSS(match(scanIDX,2),1:3),[1 1 1 0 0 0 0 0 0]',1e-1^t);
    
    % Delete bad Scan points
    Scan = Scan(scanIDX,:);

    % Rotate scanner trajectory
    Scan(:,1:3) = Trafo9(Scan(:,1:3),trafoParam);
    
    % Update complete transformation
    rotScaleNew = [trafoParam(1) 0 0; 0 trafoParam(2) 0; 0 0 trafoParam(3)]*...
                  [cos(trafoParam(6)) -sin(trafoParam(6)) 0; sin(trafoParam(6)) cos(trafoParam(6)) 0; 0 0 1]*...
                  [cos(trafoParam(5)) 0 sin(trafoParam(5)); 0 1 0; -sin(trafoParam(5)) 0 cos(trafoParam(5))]*...
                  [1 0 0; 0 cos(trafoParam(4)) -sin(trafoParam(4)); 0 sin(trafoParam(4)) cos(trafoParam(4))];          
    translation = trafoParam(7:9) + rotScaleNew*translation;
    rotScale = rotScaleNew*rotScale;

    % Plot trajectories
    pause(step)
    hold off
    plot3(GNSS(:,1),GNSS(:,2),GNSS(:,3))
    hold on
    grid on
    plot3(Scan(:,1),Scan(:,2),Scan(:,3),'k')
    
    % Reset match
    match = [];
end

%% Test transformation on original trajectory
% Plot original trajectories
figure
plot3(GNSS(:,1),GNSS(:,2),GNSS(:,3))
hold on
grid on
plot3(ScanBackup(:,1),ScanBackup(:,2),ScanBackup(:,3),'k')
pause(step)

% Transform trajectory and plot anew
for i = 1:length(ScanBackup)
   ScanBackup(i,1:3) = rotScale * ScanBackup(i,1:3)' + translation;
end
hold off
plot3(GNSS(:,1),GNSS(:,2),GNSS(:,3))
hold on
grid on
plot3(ScanBackup(:,1),ScanBackup(:,2),ScanBackup(:,3),'k')
