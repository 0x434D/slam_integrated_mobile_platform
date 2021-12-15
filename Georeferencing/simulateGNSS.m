%% Simulate GNSS Trajectory in Scanner System with Lever Arm Offset and Scanner Orientation
% ----------------------------------------------------------------------------
% Input:       traj_scan   -   Scanner Trajectory with Orientation (variable type table or matrix)
%              leverArm    -   Lever Arm between Scanner and GNSS Antenna in Scanner Coordinate System (1x3 or 3x1 matrix) 
% Output:      traj_GNSS   -   GNSS Antenna Trajectory in Scanner Coordinate System (output variable typ matching traj_input type)
% ----------------------------------------------------------------------------
% Authors:     SIMP-Project Team
% ----------------------------------------------------------------------------
% Last Modified:   05.12.2021

function [traj_GNSS] = simulateGNSS(traj_scan, leverArm)


% Check data type to match output to input trajectory
if istable(traj_scan)
    dat_type = 'table';
    traj_GNSS = traj_scan;
    traj_GNSS{:,2:4} = 0;
elseif ismatrix(traj_scan)
    dat_type = 'mat';
    traj_GNSS = traj_scan;
    traj_GNSS(:,2:4) = zeros(length(traj_scan(:,1)),3);
else
    error("Unknown input trajectory data type (only matrix or table allowed)")
end

% Check lever arm input

if size(leverArm,1) == 3
elseif size(leverArm,2) == 3
    leverArm = leverArm';
else
    error("Invalid lever arm input. Please give a 1x3 or 3x1 numeric array");
end

rotz = [0 -1 0; 1 0 0; 0 0 1];

for i=1:size(traj_scan,1)
    switch dat_type
        case 'mat'
            rotMat = quat2rotm(traj_scan(i,5:8));
            leverArm_i = rotz * rotMat * leverArm;
            traj_GNSS(i,2:4) = traj_scan(i,2:4) + leverArm_i';
        case 'table'
            rotMat = quat2rotm(traj_scan{i,5:8});
            leverArm_i = rotz * rotMat * leverArm;
            traj_GNSS{i,2:4} = traj_scan{i,2:4} + leverArm_i';
    end
end
