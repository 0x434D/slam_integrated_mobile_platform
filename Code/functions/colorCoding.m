function pointColors = colorCoding(ptCloud,ptTimes,firstTime,timeOffset,gnss,scan,rotScale,iDim,img,frameTimes)
% Uses captured 3D-GoPro images of a run to colorize the point cloud that 
% was previously georeferenced.
% -------------------------------------------------------------------------
% Autor: SIMP-Project Team
% -------------------------------------------------------------------------
% To connect the images and the point cloud, we use the gps timestamps in a
% combined time frame. As a first step we calculate all corresponding
% points for one image based on the frame time and safe the corresponding
% indices. Thus, each point is only colored once by one specific image.
% Next, we prepare the image by first calculating the orientation (azimuth)
% from the scan quaternions. We also derive the position of the GoPro from
% the GNSS antenna position and a small z-offset (GoPro is below antenna).
% Finally we calculate spheric coordinates of each point with respect to
% the camera position in a flat-earth frame and from that derive two angles
% to get a pixel position and corresponding RGB.
% -------------------------------------------------------------------------
% Input:    See variable names from Georeferencing
% Output:   pointColors = RGB colors for all points (uint8)
% -------------------------------------------------------------------------

%% Define a mask for points on the scanner or antenna plattform
mask = load('SIMPmask.mat').SIMPmask;
dsFactor = round(size(mask,1)/iDim(1));    	% determine downsample factor
mask = mask(1:dsFactor:end,1:dsFactor:end);

%% Connect image and scan data via timestamps
fprintf('\tConnect image and scan data via timestamps\n')

% Transform point times into MATLAB datetime [seconds]
ptTimes = ptTimes - firstTime + timeOffset;

% Loop over all images to find the closest scan-point index for each frame
for i = 1:length(frameTimes)
    [~,idx] = min(abs(ptTimes-frameTimes(i)));
    pointIdx(i,1) = idx;
end

% Define the scan points for each image with indices of bounds
% First image: [bounds(1) bounds(2)], Second: [bounds(2) bounds(3)] ...
idxBounds = [1;pointIdx(1:end-1)+floor(diff(pointIdx)/2);length(ptTimes)];

%% Prepare spatial data
fprintf('\tPrepare spatial data\n')

% Switch x- and y-axis and flip z-axis of all spatial data because
% flat-earth convention is not suitable for colorCoding
gnss = [gnss(:,1) gnss(:,3) gnss(:,2) -gnss(:,4)];
scan = [scan(:,1) scan(:,3) scan(:,2) -scan(:,4) scan(:,5:8)];
ptCloud = [ptCloud(:,2) ptCloud(:,1) -ptCloud(:,3)];

%% Loop over all images
fprintf('\tLoop over all images\n')

% Create variable to safe colors with a flag that indicates whether the
% point color is inside the mask region (vehicle setup --> false color)
pointColors = uint8(zeros(size(ptTimes,1),4));

for i = 1:size(frameTimes,1)
%% Get orientation and position of current frame
% Add experimental time offset to frame times because the actual recording 
% time is always a little earlier
time = frameTimes(i)-0.7;

% Function to get current orientation from GNSS trajectory 
[orient,camOrig] = tra2ori(gnss,scan,time,rotScale);

% Load corresponding scan points for image
points = ptCloud(idxBounds(i):idxBounds(i+1),:);

%% Find RGB for each scan point
% Define origin and calculate the vector of each scan point to origin
camOrig = camOrig - [0 0 0.13];	% camera position (13 cm below antenna)
diffVec = points-camOrig;

% Spheric coordinates (two angles) from cartestian coordinates
[th,phi,~] = cart2sph(diffVec(:,1),diffVec(:,2),diffVec(:,3));

% Adjust orientation with vehicle trajectory 
th = th+pi-(3*pi/2-orient);     % this if scanner is at the sides
% th = th+pi-(pi/2-orient);     % this if scanner is in the middle

% Spheric angles to MATLAB plot pixel positions
% (-1 and +1 to assure result to be within [1 1920] or [1 1080])
xImg = round(mod(th,2*pi)*(iDim(2)-1)/(2*pi))+1;
yImg = round((pi/2-phi)*(iDim(1)-1)/pi)+1;

% Extract RGB color of the pixel(s) from 4D picture matrix
one = ones(length(xImg),1);
rgb = [img(sub2ind(size(img), yImg, xImg, one*1, one*i))...
       img(sub2ind(size(img), yImg, xImg, one*2, one*i))...
       img(sub2ind(size(img), yImg, xImg, one*3, one*i))];
rgb = uint8(rgb);              % convert to uint8 for .las color

% Set points outside the mask to black to change later
in = mask(sub2ind(size(mask),yImg,xImg));
rgb(~in,:) = repmat([0 0 0],[size(rgb(~in,:),1),1]);

% Add up detected point colors for all images
pointColors(idxBounds(i):idxBounds(i+1),:) = [rgb in];

%% Showcase
% % Show image (pick with i) and current scan points to be colored
% if i == 30
%     figure 
%     imshow(img(:,:,:,i))
%     hold on
%     scatter(xImg,yImg,7,'b','filled')
%     drawnow
%     
%     figure
%     pcshow(ptCloud(idxBounds(i):idxBounds(i+1),:),rgb)
% end
end

%% Assign new color for points on image mask via spatial neighbourhood
fprintf('\tAssign new color to occluded points\n')

% Set voxel parameters
voxelLength = 1;        % [m]
posMin = zeros(3,1);
posMax = zeros(3,1);
for i=1:3
    posMin(i) = floor(min(ptCloud(:,i))/voxelLength)*voxelLength;
    posMax(i) =  ceil(max(ptCloud(:,i))/voxelLength)*voxelLength;
end

% Edges for discretization
edgesX = posMin(1):voxelLength:posMax(1);
edgesY = posMin(2):voxelLength:posMax(2);
edgesZ = posMin(3):voxelLength:posMax(3);

% Discretization
bins = zeros(size(ptCloud));
bins(:,1) = discretize(ptCloud(:,1),edgesX);
bins(:,2) = discretize(ptCloud(:,2),edgesY);
bins(:,3) = discretize(ptCloud(:,3),edgesZ);

% Sort bins
[bins, sortIdx] = sortrows(bins,[1 2 3]);
        
% Check all voxels
bin = zeros(1,3);
idxA = 1;
n = size(bins,1);
noColor = 0;
for i =  1:n
    idxE = i;
    if any(bin ~= bins(i,:))
        bin = bins(i,:);
        idxA = i;
    end
    if i == n || any(bins(i,:) ~= bins(i+1,:))

        % Find points that need recoloring and assign new color
        change = find(pointColors(sortIdx(idxA:idxE),4) == 0);

        % If points in voxel need recoloring
        if ~isempty(change)
            pool = pointColors(sortIdx(idxA:idxE),:);

            % If voxel only contains uncolored points
            if size(pool,1) == size(change,1)
                 noColor = noColor + 1;

            % If amount of already colored points is <= 20 take mean of all
            elseif (size(pool,1)-size(change,1)) <= 20
                for c = 1:size(change,1)
                    pointColors(sortIdx(idxA + change(c) - 1),:) = uint8([round(mean(pool(pool(:,4) ~= 0,1:3),1)) 0]);
                end

            % Else find the closest 20 points for recoloring
            else
                poolIDX = find(pointColors(sortIdx(idxA:idxE),4) == 1);
                poolCloud = pointCloud(ptCloud(sortIdx(idxA + poolIDX - 1),:));
                for c = 1:size(change,1)
                    [closeIDX,~] = findNearestNeighbors(poolCloud,ptCloud(sortIdx(idxA + change(c) - 1),:),20);
                    pointColors(sortIdx(idxA + change(c) - 1),:) = uint8([round(mean(pool(poolIDX(closeIDX),1:3),1)) 0]);
                end
            end
        end
    end
end
fprintf('\t--> Total of %.0f points could not be colorized (now black)\n', noColor)

%% Plot whole point cloud
% f1 = figure;
% set(f1, 'Position', [400 50 1400 800])
% view([0 25])
% pcshow(ptCloud,pointColors(:,1:3));

% gif('SchlossV01.gif','DelayTime',1/40,'overwrite',true)
% for i = 0:360
%    view([i 25])
%    gif
% end

end