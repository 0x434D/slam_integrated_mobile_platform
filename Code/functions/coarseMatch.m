function [Scan,rotS,trans] = coarseMatch(GNSS, Scan, tOff)
% Function performs a coarse trajectory match for the provided GNSS and
% Scan trajectory and returns the transformed trajectory as well as the
% transformation parameters
% -------------------------------------------------------------------------
% Autor: SIMP-Project Team
% -------------------------------------------------------------------------
% Trajectories have to have matched times! The coarse matching is performed
% by estimating the azimuth difference between the furthest point from the
% start in both trajectories and calculating the offset between the start
% points of both routes.
% -------------------------------------------------------------------------
% Input:    GNSS    = GNSS trajectory [time [s], X [m], Y [m], Z [m]]
%           Scan    = Scan trajectory [time [s], X [m], Y [m], Z [m] ...]
%           tOff    = Time offset between GNSS and Scan trajectory [s]
% Output:   Scan    = Scan trajectory [time [s], X [m], Y [m], Z [m] ...]
%           rotS    = combined rotation and scale of coarse transformation
%           trans   = translation of coarse transformation [m]
% -------------------------------------------------------------------------

% Get time offset and mean time step size of Scan trajectory
ScanTSS = mean(diff(Scan(:,1)));

% Estimate furthest point from start in GNSS trajectory
[~,idxGNSS] = max(vecnorm((GNSS(:,2:4)-GNSS(1,2:4)),2,2));

% Get furthest point from start in Scan trajectory
idxScan = round((GNSS(idxGNSS,1)-tOff)/ScanTSS);

% Estimate azimuth of both trajectories with first and furthest point
aziGNSS = mod(atan2(GNSS(idxGNSS,3)-GNSS(1,3),GNSS(idxGNSS,2)-GNSS(1,2)),2*pi);
aziScan = mod(atan2(Scan(idxScan,3)-Scan(1,3),Scan(idxScan,2)-Scan(1,2)),2*pi);

% Estimate rough z-axis rotation between trajectories
zRot = aziScan-aziGNSS;

% Rotate scanner trajectory around z
trafoCoarse = [1 1 1 0 0 -zRot 0 0 0]';
Scan(:,2:4) = Trafo9(Scan(:,2:4),trafoCoarse);

% Find initial offset and add to scan trajectory
trans = (GNSS(1,2:4)-Scan(1,2:4))';
Scan(:,2:4) = Scan(:,2:4)+trans';

% Combine rotations
rotS1 = [trafoCoarse(1) 0 0; 0 trafoCoarse(2) 0; 0 0 trafoCoarse(3)]*...
       [cos(trafoCoarse(6)) -sin(trafoCoarse(6)) 0; sin(trafoCoarse(6)) cos(trafoCoarse(6)) 0; 0 0 1]*...
       [cos(trafoCoarse(5)) 0 sin(trafoCoarse(5)); 0 1 0; -sin(trafoCoarse(5)) 0 cos(trafoCoarse(5))]*...
       [1 0 0; 0 cos(trafoCoarse(4)) -sin(trafoCoarse(4)); 0 sin(trafoCoarse(4)) cos(trafoCoarse(4))];
rotS = rotS1;
end

