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
leverArm = [-0.019 0.127 0.191]';

% Load GNSS trajectory (format: [time [s], X [m], Y [m], Z [m]])
load('Data\testGNSS_1.mat','POS','POS_label','GPS2','GPS2_label');
GNSS = lla2ecef(POS(:,3:5));            % transform to ECEF [m]
GNSS = [POS(:,2)*1e-6 GNSS];            % add time in [s]

% Load Scanner trajectory (format: [time [s], X [m], Y [m], Z [m] ...])
ScanRaw = load('Data\testSCAN_1.txt');

% Add lever arm to scan trajectory with orientation from scanner
Scan = simulateGNSS(ScanRaw, leverArm);
ScanBackup = Scan;                        % save original Scan trajectory

% Reduce scan trajectory times 
Scan(:,1) = Scan(:,1)-Scan(1,1);

% Delete first 50 GNSS measurements to compensate time difference (experimental!)
GNSS = GNSS(50:end,:);

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

% % Plot trajectories
% figure
% plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4))
% hold on
% grid on
% plot3(Scan(:,2),Scan(:,3),Scan(:,4),'k')

%% Accurate trajectory match
[Scan,rotScale,translation] = accurateMatch(GNSS, Scan, timeOffset, rotScale, translation, 7);

% Add initial offset
translation = translation + offset';
GNSS(:,2:4) = GNSS(:,2:4) + offset;

%% Test transformation on original trajectory
% Transform trajectory and plot anew
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4))
hold on 
view([60 55])
grid on
for i = 1:length(ScanBackup)
   ScanBackup(i,2:4) = rotScale * ScanBackup(i,2:4)' + translation;
end
plot3(ScanBackup(:,2),ScanBackup(:,3),ScanBackup(:,4),'k')
% print('-dpng','-r200',"AfterTrafo.png")

% TODO: Transformationskombination stimmt noch nicht ganz...
%       Ist die Schätzung wirklich gut? Sieht so aus als sollte man noch
%       etws rotieren können für den besten match