function [orient,pos] = tra2ori(gnss,time,rotScale)
% Function calculates the orientation and antenna position of the vehicle 
% based on the given trajectory and time.
% -------------------------------------------------------------------------
% Autor: SIMP-Project Team
% -------------------------------------------------------------------------
% It is important that the vehicle is moving at the given time, otherwise
% the orientation can not be calculated. In this case, the next closest
% orientation is calculated by increasing the number of observations.
% -------------------------------------------------------------------------
% Input:    gnss    = GNSS trajectory [time [s], X [m], Y [m], Z [m]]
%           time    = time of image
% Output:   orient  = orientation of the vehicle with respect to the gnss
%                     refernce frame (here: flat earth frame) [rad]
%           pos     = position of the antenna at the given time [m]
% -------------------------------------------------------------------------

% Check if given time is within GNSS trajectory time
if time < gnss(1,1) || time > gnss(end,1)
    error('Image time is not within the given trajectory times!')
end

% Find closest index in GNSS trajectory that matches time of image
[~,idx] = min(abs(gnss(:,1)-repmat(time,size(gnss(:,1)))));

% Define a window for observations to calculate orientation
deltaT = mean(diff(gnss(:,1)));
sizeWin = round(1/deltaT);              % this corresponds to one second
if idx-sizeWin < 1
    idxLB = 1;
else
    idxLB = idx-sizeWin;                % lower bound
end
if idx+sizeWin > size(gnss,1)
    idxUB = size(gnss,1);
else
    idxUB = idx+sizeWin;                % upper bound
end

% Calculate help points (X,Y) for azimuth orientation calculation (clockwise)
helpLB = mean(gnss(idxLB:idx,2:3),1);
helpUB = mean(gnss(idx:idxUB,2:3),1);
orient = mod(atan2(helpUB(1)-helpLB(1),helpUB(2)-helpLB(2)),2*pi);

% Also save GNSS antenna position at idx
pos = gnss(idx,2:4);

% Check if orientation calculation was reasonable
if norm(helpLB-helpUB) < 0.01
    warning('Could not calculate orientation due to no movement. Using next closest orientation.')
    
    % Extend the window for observations to use for orientation calculation
    extend = 2;
    while norm(helpLB-helpUB) < 0.01
        if idx-extend*sizeWin < 1
            idxLB = 1;
        else
            idxLB = idx-extend*sizeWin;      	% lower bound
        end
        if idx+extend*sizeWin > size(gnss,1)
            idxUB = size(gnss,1);
        else
            idxUB = idx+extend*sizeWin;        	% upper bound
        end
        helpLB = mean(gnss(idxLB:idx,2:3),1);
        helpUB = mean(gnss(idx:idxUB,2:3),1);  
        extend = extend+1;
    end
    orient = mod(atan2(helpUB(1)-helpLB(1),helpUB(2)-helpLB(2)),2*pi);
end

% Directly get the azimuth from quaternions
% DCM11 = gnss(idx,5)^2+gnss(idx,6)^2-gnss(idx,7)^2-gnss(idx,8)^2;
% DCM21 = 2*(gnss(idx,6)*gnss(idx,7)-gnss(idx,8)*gnss(idx,5));
% orient = mod(atan2(DCM21,DCM11),2*pi);
rotMat = rotScale*quat2rotm(gnss(idx,5:8))*[0 0 1; -1 0 0; 0 -1 0];
% orient = 2*pi-mod(atan2(rotMat(1,1),rotMat(1,2)),2*pi);
orient = 2*pi-mod(atan2(rotMat(1,2),rotMat(1,1))+pi/2,2*pi);


% plot3(gnss(idxLB:idxUB,2),gnss(idxLB:idxUB,3),gnss(idxLB:idxUB,4),'r','LineWidth',2)
% drawnow
end

