function [orient,pos] = tra2ori(gnss,scan,time,rotScale)
% Function calculates the orientation and antenna position of the vehicle 
% from scan trajectory and gnss trajectory respectively.
% -------------------------------------------------------------------------
% Autor: SIMP-Project Team
% -------------------------------------------------------------------------
% The function matches the time to the given scan and gnss trajectory to
% compute position and orientation (azimuth) of the camera in a flat earth
% frame. If the gnss position is flawed (for example due to trees), the
% position can be switched to be estimated by the scan trajectory aswell.
% -------------------------------------------------------------------------
% Input:    gnss        = GNSS trajectory [time [s], X [m], Y [m], Z [m]]
%           scan        = scan trajectory [time [s], X [m], Y [m], Z [m]]
%           time        = time of image
%           rotScale    = rotation/scale matrix of the accurate matching
% Output:   orient  = orientation of the vehicle with respect to the
%                     refernce frame (here: flat earth frame) [rad]
%           pos     = position of the antenna at the given time [m]
% -------------------------------------------------------------------------

% Check if given time is within trajectory time
if time < gnss(1,1) || time > gnss(end,1)
    warning('Image time is not within the given trajectory times! Using next closest time.')
    if time < gnss(1,1)
        time = gnss(1,1);
    else
        time = gnss(end,1);
    end
end

% Find closest index in GNSS/Scan trajectory that matches time of image
[~,idxG] = min(abs(gnss(:,1)-repmat(time,size(gnss(:,1)))));
[~,idxS] = min(abs(scan(:,1)-repmat(time,size(scan(:,1)))));

% Save GNSS antenna position at idxG
pos = gnss(idxG,2:4);

% Get orientation from scan trajectory quaternions at idxS
rotMat = rotScale*quat2rotm(scan(idxS,5:8))*[0 0 1; -1 0 0; 0 -1 0];
orient = 2*pi-mod(atan2(rotMat(1,2),rotMat(1,1))+pi/2,2*pi);
end

