%% Georeferencing - Trajectory matching
% Author: SIMP Team
clearvars
close all
format longG
clc
fprintf('Georeferencing - Trajectory matching\n')

addpath('icp')
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
Scan = simulateGNSS(ScanRaw, leverArm);
ScanBackup = Scan;                        % save original Scan trajectory

% Reduce scan trajectory times 
Scan(:,1) = Scan(:,1)-Scan(1,1);

% Delete first 200 GNSS measurements to compensate time difference (experimental!)
GNSS = GNSS(200:end,:);

% Get mean time step size
ScanTSS = mean(diff(Scan(:,1)));
GNSSTSS = mean(diff(GNSS(:,1)));

% Match times
timeOffset = GNSS(1,1)-Scan(1,1);
Scan(:,1) = Scan(:,1)+timeOffset;

% Reduce coordinates for numric stability
offset = mean(GNSS(:,2:4));
GNSS(:,2:4) = GNSS(:,2:4)-offset;
% Scan(:,2:4) = Scan(:,2:4)-offset;

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

% Add initial offset
translation = translation + offset';
GNSS(:,2:4) = GNSS(:,2:4) + offset;

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
legend('GNSS','SCAN','Location','NorthWest')
title('Combined Transformation (Coarse + Accurate)')
view([90 90])
% print('-dpng','-r200',"AccurateTrafo_Ugly.png")

% TODO: 
%       - irgendwie winkelbild noch benutzen?
%       - Gewichtsmatrix --> Irgendwann später
%       - Scan mehr Punkte oder GNSS meh Punkte unabhängig
%       - Flat Earth Transformation statt ECEF wegen der Drehung --> So besser
%         matchbar --> Am Ende dann flat2lla und lla2ecef (etwas umständlich aber naja)
%       - Z-Koordinate ist Hauptproblem, wie erwartbar --> Vll runter gewichten
%         in P matrix?