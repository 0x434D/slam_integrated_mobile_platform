%% Main file to import data and test the function findTimeDelay.m

clear
close all
clc

%% Import Data

addpath('..\');

% Switch between test run 1 and 2
testCase = 'First1';
% testCase = 'First2';
% testCase = 'RTK1'; % Do not use yet, GNSS trajectory missing
% testCase = 'RTK2';

% Set step size
step_size = 0.1;

% lever arm

lever = [-1.0967 13.3986 20.201] .* 1e-2;

switch testCase
    case 'First1'
        gnss_path = "..\Data\GNSSFirstRun.txt";
        scan_path = "..\Data\ScanFirstRun.txt";
    case 'First2'
        gnss_path = "..\Data\GNSSSecondRun.txt";
        scan_path = "..\Data\ScanSecondRun.txt";
    case 'RTK1'
        gnss_path = "..\Data\testGNSS_RTK1.mat";
        scan_path = "..\Data\ScanRTKFirstRun.txt";
    case 'RTK2'
        gnss_path = "..\Data\testGNSS_RTK2.mat";
        scan_path = "..\Data\ScanRTKSecondRun.txt";
end

if ~strcmp(testCase,'RTK2')
    traj_gnss = importdata(gnss_path);
    traj_gnss = traj_gnss.data;
else
    traj_gnss = load(gnss_path,'GPS2');
    traj_gnss = traj_gnss.GPS2(:,[2, 8:10]);
%     traj_gnssCompOrig = traj_gnss;
    llo_gnss = mean(traj_gnss(:,2:3),1);
    traj_flat = lla2flat(traj_gnss(:,2:4), llo_gnss, 0, 0);
    traj_gnss = [traj_flat(:,2), traj_flat(:,1), traj_flat(:,3), traj_gnss(:,1)];
%     traj_gnssCompT = flat2lla([traj_gnss(:,2),traj_gnss(:,1),traj_gnss(:,3)], llo_gnss, 0, 0);
end

traj_scan = importdata(scan_path);
traj_scan = traj_scan.data;

traj_gnss(:,4) = traj_gnss(:,4) * 1e-6;    % time from mikroseconds to seconds
gnss_offsett = mean(traj_gnss(:,1:3),1);
traj_gnss(:,1:3) = traj_gnss(:,1:3) - gnss_offsett;

traj_simGnss = simulateGNSS(traj_scan,lever);

figure;
scatter(traj_simGnss(:,2),traj_simGnss(:,3),'b');
hold on
scatter(traj_scan(:,2),traj_scan(:,3),'r');
title('Simulated GNSS trajectory')
legend('simGnss','Scanner');

[dt,dtInv] = findTimeDelay(traj_simGnss, traj_gnss, step_size);

traj_match = matchTrajByTime(traj_scan,traj_gnss,dt);

figure
plot3(traj_match(:,1),traj_match(:,2),traj_match(:,3),'r')
hold on 
axis equal
grid on
plot3(traj_gnss(:,1),traj_gnss(:,2),traj_gnss(:,3),'b')
legend('Time matched','GNSS')
title('Time matched trajectory vs GNSS trajectory')

%%
[test,rotS,trans] = TimCoarseMatch([traj_match(:,4),traj_match(:,1:3)],traj_scan);

matchPlot(1:3:3*length(test(1:50:end,:)),:) = test(1:50:end,2:4);
matchPlot(2:3:end+1,:) = traj_match(1:50:end,1:3);
matchPlot(3:3:end+1,:) = NaN;

figure
plot3(test(:,2),test(:,3),test(:,4),'.-b')
hold on 
axis equal
grid on
plot3(traj_match(:,1),traj_match(:,2),traj_match(:,3),'.-r')
plot3(matchPlot(:,1),matchPlot(:,2),matchPlot(:,3),'.-g')
legend('Transformed scan','GNSS')
title('Coarse Match')

%%
trafo9 = TimEst9Trafo3D(test(:,2:4),traj_match(:,1:3),[1 1 1 0 0 0 0 0 0]',1e-5);

traj_scanTrans = Trafo9(test(:,2:4),trafo9);

%%
figure
plot3(traj_gnss(:,1),traj_gnss(:,2),traj_gnss(:,3),'-b')
hold on 
axis equal
grid on
plot3(traj_scanTrans(:,1),traj_scanTrans(:,2),traj_scanTrans(:,3),'-r')
legend('GNSS','Transformed scan')
title('Transformierte Scan Trajektorie vs GNSS')

std_Trafo = vecnorm(vecnorm(test(:,2:4)-traj_match(:,1:3),2,2))/sqrt(length(test(:,1)));
std_TrafoX = vecnorm(vecnorm(test(:,2)-traj_match(:,1),2,2))/sqrt(length(test(:,1)));
std_TrafoY = vecnorm(vecnorm(test(:,3)-traj_match(:,2),2,2))/sqrt(length(test(:,1)));
std_TrafoZ = vecnorm(vecnorm(test(:,4)-traj_match(:,3),2,2))/sqrt(length(test(:,1)));
fprintf("Standard deveation XYZ: \t%.4f\nStandard deveation X: \t\t%.4f\nStandard deveation Y: \t\t%.4f\nStandard deveation Z: \t\t%.4f\n", std_Trafo, std_TrafoX, std_TrafoY, std_TrafoZ)

