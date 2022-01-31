function del = removeMovingObjects(xyz, t, voxelLength, timeDiff)
% -------------------------------------------------------------------------
% removeMovingObjects  tells which point belong to moving objects
%       if the points contained in a voxel have all been measured in a
%       short period of time and never again, then there was probably some
%       movement and the points can be deleted.
%       actual deletion has to be carried out by vector(del) = [], or
%       matrix(del,:) = [];
% -------------------------------------------------------------------------
% Authors:     SIMP-Project Team
% -------------------------------------------------------------------------
% inputs:
%   xyz        : x-,y, and z-coordinates as matrix. number of rows == number of points
%   t          : time of observation for each point
%   gridLength : length of voxel grid
%   timeDiff   : critical time difference
% output:
%   del       : boolean vector telling which points can be deleted
% -------------------------------------------------------------------------

% computes min and max values of x, y and z as multiple of voxelLength
posMin = zeros(3,1);
posMax = zeros(3,1);
for i=1:3
    posMin(i) = floor(min(xyz(:,i))/voxelLength)*voxelLength;
    posMax(i) =  ceil(max(xyz(:,i))/voxelLength)*voxelLength;
end

% edges for discretization
edgesX = posMin(1):voxelLength:posMax(1);
edgesY = posMin(2):voxelLength:posMax(2);
edgesZ = posMin(3):voxelLength:posMax(3);

bins = zeros(size(xyz));
% discretization
bins(:,1) = discretize(xyz(:,1),edgesX);
bins(:,2) = discretize(xyz(:,2),edgesY);
bins(:,3) = discretize(xyz(:,3),edgesZ);

% sort bins
[bins, sortIdx] = sortrows(bins,[1 2 3]);

% sort time
t = t(sortIdx);

% check all voxels
bin = zeros(1,3);
idxA = 1;
n = size(bins,1);
del = false(n,1);
for i =  1:n

    if any(bin ~= bins(i,:))
        bin = bins(i,:);
        idxA = i;
    end
    if i == n || any(bins(i,:) ~= bins(i+1,:))
        idxE = i;
        % no comparison possible with only one point per voxel
        if idxE-idxA > 0 && (max(t(idxA:idxE)) - min(t(idxA:idxE)) < timeDiff)
            del(idxA:idxE) = true;
        end
    end
end

[~,revSortIdx] = sort(sortIdx);
del = del(revSortIdx);

end