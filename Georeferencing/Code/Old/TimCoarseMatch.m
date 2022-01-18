function [Scan,rotS,trans] = TimCoarseMatch(GNSS, Scan)
% Function performs a coarse trajectory match for the provided GNSS and
% Scan trajectory and returns the transformed trajectory as well as the
% transformation parameters
% -------------------------------------------------------------------------
% Autor: SIMP-Project Team
% -------------------------------------------------------------------------
% Trajectories have to have matched times!
% -------------------------------------------------------------------------
% Input:    GNSS    = GNSS trajectory [time [s], X [m], Y [m], Z [m]]
%           Scan    = Scan trajectory [time [s], X [m], Y [m], Z [m] ...]
%           tOff    = Time offset between GNSS and Scan trajectory [s]
% Output:   Scan    = Scan trajectory [time [s], X [m], Y [m], Z [m] ...]
%           rotS    = combined rotation and scale of coarse transformation
%           trans   = translation of coarse transformation [m]
% -------------------------------------------------------------------------
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4),'b')
hold on 
axis equal
grid on
plot3(Scan(:,2),Scan(:,3),Scan(:,4),'r')

% Get time offset and mean time step size of Scan trajectory
ScanTSS = mean(diff(Scan(:,1)));

% Estimate furthest point from start in GNSS trajectory and plot
[~,idxGNSS] = max(vecnorm((GNSS(:,2:4)-GNSS(1,2:4)),2,2));
scatter3(GNSS([1,idxGNSS],2),GNSS([1,idxGNSS],3),GNSS([1,idxGNSS],4),'b', 'filled')

% Get furthest point from start in Scan trajectory and plot
% [~,idxScan] = max(vecnorm((Scan(:,1:3)-Scan(1,1:3)),2,2));
idxScan = round((GNSS(idxGNSS,1)-GNSS(1,1))/ScanTSS);
scatter3(Scan([1,idxScan],2),Scan([1,idxScan],3),Scan([1,idxScan],4),'r','filled')

% Estimate azimuth of both trajectories with first and furthest point
aziGNSS = mod(atan2(GNSS(idxGNSS,3)-GNSS(1,3),GNSS(idxGNSS,2)-GNSS(1,2)),2*pi);
aziScan = mod(atan2(Scan(idxScan,3)-Scan(1,3),Scan(idxScan,2)-Scan(1,2)),2*pi);

% Estimate y-rotation with furthest point (because ECEF is tilted)
yrotGNSS = mod(atan2(GNSS(idxGNSS,4)-GNSS(1,4),GNSS(idxGNSS,2)-GNSS(1,2)),2*pi);
yrotScan = mod(atan2(Scan(idxScan,4)-Scan(1,4),Scan(idxScan,2)-Scan(1,2)),2*pi);

% % % Estimate x-rotation with furthest point (might be added later)
% xrotGNSS = mod(atan2(GNSS(idxGNSS,4)-GNSS(1,4),GNSS(idxGNSS,3)-GNSS(1,3)),2*pi);
% xrotScan = mod(atan2(Scan(idxScan,4)-Scan(1,4),Scan(idxScan,3)-Scan(1,3)),2*pi);

% Estimate rough z-axis rotation between trajectories
zRot = aziScan-aziGNSS;
yRot = yrotScan-yrotGNSS;
% xRot = xrotScan-xrotGNSS;

% Rotate scanner trajectory
trafoCoarse = [1 1 1 0 0 -zRot 0 0 0]';
trafoECEF = [1 1 1 0 yRot 0 0 0 0]';
% trafoECEF2 = [1 1 1 xRot 0 0 0 0 0]';
Scan(:,2:4) = Trafo9(Scan(:,2:4),trafoCoarse);  % rotate around z

% Plot trajectories
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4),'b')
hold on
view([60 55])
grid on
plot3(Scan(:,2),Scan(:,3),Scan(:,4),'r')
scatter3(Scan(idxScan,2),Scan(idxScan,3),Scan(idxScan,4),'r','filled')
scatter3(GNSS(idxGNSS,2),GNSS(idxGNSS,3),GNSS(idxGNSS,4),'b','filled')

Scan(:,2:4) = Trafo9(Scan(:,2:4),trafoECEF);    % rotate around y

% Plot trajectories
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4),'b')
hold on
view([60 55])
grid on
plot3(Scan(:,2),Scan(:,3),Scan(:,4),'r')
scatter3(Scan(idxScan,2),Scan(idxScan,3),Scan(idxScan,4),'r','filled')
scatter3(GNSS(idxGNSS,2),GNSS(idxGNSS,3),GNSS(idxGNSS,4),'b','filled')
% Scan(:,2:4) = Trafo9(Scan(:,2:4),trafoECEF2);    % rotate around x
% Scan(:,2:4) = Trafo9(Scan(:,2:4),[1 1 1 -0.1 0 0 0 0 0]');    % rotate around y

% % Estimate x-rotation with furthest point (might be added later)
xrotGNSS = mod(atan2(GNSS(idxGNSS,4)-GNSS(1,4),GNSS(idxGNSS,3)-GNSS(1,3)),2*pi);
xrotScan = mod(atan2(Scan(idxScan,4)-Scan(1,4),Scan(idxScan,3)-Scan(1,3)),2*pi);
xRot = xrotScan-xrotGNSS;
% ADD ROTATION!!!!
trafoECEF2 = [1 1 1 -xRot 0 0 0 0 0]';
Scan(:,2:4) = Trafo9(Scan(:,2:4),trafoECEF2);    % rotate around x



% Find initial offset and add to scan trajectory
trans = (GNSS(1,2:4)-Scan(1,2:4))';
Scan(:,2:4) = Scan(:,2:4)+trans';

% Plot trajectories
figure
plot3(GNSS(:,2),GNSS(:,3),GNSS(:,4),'b')
hold on
view([60 55])
grid on
plot3(Scan(:,2),Scan(:,3),Scan(:,4),'r')
scatter3(Scan(idxScan,2),Scan(idxScan,3),Scan(idxScan,4),'r','filled')
scatter3(GNSS(idxGNSS,2),GNSS(idxGNSS,3),GNSS(idxGNSS,4),'b','filled')

% Combine rotations
rotS1 = [trafoCoarse(1) 0 0; 0 trafoCoarse(2) 0; 0 0 trafoCoarse(3)]*...
       [cos(trafoCoarse(6)) -sin(trafoCoarse(6)) 0; sin(trafoCoarse(6)) cos(trafoCoarse(6)) 0; 0 0 1]*...
       [cos(trafoCoarse(5)) 0 sin(trafoCoarse(5)); 0 1 0; -sin(trafoCoarse(5)) 0 cos(trafoCoarse(5))]*...
       [1 0 0; 0 cos(trafoCoarse(4)) -sin(trafoCoarse(4)); 0 sin(trafoCoarse(4)) cos(trafoCoarse(4))];
rotS2 = [trafoECEF(1) 0 0; 0 trafoECEF(2) 0; 0 0 trafoECEF(3)]*...
       [cos(trafoECEF(6)) -sin(trafoECEF(6)) 0; sin(trafoECEF(6)) cos(trafoECEF(6)) 0; 0 0 1]*...
       [cos(trafoECEF(5)) 0 sin(trafoECEF(5)); 0 1 0; -sin(trafoECEF(5)) 0 cos(trafoECEF(5))]*...
       [1 0 0; 0 cos(trafoECEF(4)) -sin(trafoECEF(4)); 0 sin(trafoECEF(4)) cos(trafoECEF(4))];
rotS = rotS2*rotS1;

end

