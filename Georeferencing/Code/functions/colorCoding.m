function pointColors = colorCoding(pointCloud,pointTimes,firstTime,timeOffset,gnss,scan,rotScale,iDim,img,frameTimes)
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
pointTimes = pointTimes - firstTime + timeOffset;

% Loop over all images to find the closest scan-point index for each frame
for i = 1:length(frameTimes)
    [~,idx] = min(abs(pointTimes-frameTimes(i)));
    pointIdx(i,1) = idx;
end

% Define the scan points for each image with indices of bounds
% First image: [bounds(1) bounds(2)], Second: [bounds(2) bounds(3)] ...
idxBounds = [1;pointIdx(1:end-1)+floor(diff(pointIdx)/2);length(pointTimes)];

%% Prepare spatial data
fprintf('\tPrepare spatial data\n')

% Switch x- and y-axis and flip z-axis of all spatial data because
% flat-earth convention is not suitable for colorCoding
gnss = [gnss(:,1) gnss(:,3) gnss(:,2) -gnss(:,4)];
scan = [scan(:,1) scan(:,3) scan(:,2) -scan(:,4) scan(:,5:8)];
pointCloud = [pointCloud(:,2) pointCloud(:,1) -pointCloud(:,3)];

%% Loop over all images
fprintf('\tLoop over all images\n')

% Create variable to safe colors with a flag that indicates whether the
% point color is inside the mask region (vehicle setup --> false color)
pointColors = uint8(zeros(size(pointTimes,1),4));

for i = 1:size(frameTimes,1)
%% Get orientation and position of current frame
% Add experimental time offset to frame times because the actual recording 
% time is always a little earlier
time = frameTimes(i)-0.7;

% Function to get current orientation from GNSS trajectory 
[orient,camOrig] = tra2ori(gnss,scan,time,rotScale);

% Load corresponding scan points for image
points = pointCloud(idxBounds(i):idxBounds(i+1),:);

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

% Set points outside the mask to purple (uncommon color) to change later
in = mask(sub2ind(size(mask),yImg,xImg));
rgb(~in,:) = repmat([120 0 255],[size(rgb(~in,:),1),1]);

% Add up detected point colors for all images
pointColors(idxBounds(i):idxBounds(i+1),:) = [rgb in];

%% Showcase
% % Show rotated image (pick with i) and current scan points to be colored
% if i == 276
%     figure 
%     imshow(img(:,:,:,i))
%     hold on
%     scatter(xImg,yImg,7,'b','filled')
%     drawnow
%     
%     figure
%     pcshow(pointCloud(idxBounds(i):idxBounds(i+1),:),rgb)
% end
end

%% Assign new color for points on image mask via spatial neighbourhood
fprintf('\tAssign new color to occluded points\n')

% Sort all points (and colors) by distance to first point
pointDist = vecnorm(pointCloud-pointCloud(1,1:3),2,2);
[~,dIDX] = sort(pointDist);
colorSort = pointColors(dIDX,1:4);

% List points for which new colors will be determined
change = find(pointColors(:,4) == 0);

% Get inverse index list of pointDist vector to find change points there
[~,dIDXinv] = sort(dIDX);

% Loop over all points
for p = 1:size(change,1)
    % Find the point in sorted color vector and check limits
    changeIDX = dIDXinv(change(p));
    if changeIDX+10 > size(colorSort,1)
        changeIDX = changeIDX - 10;
    elseif changeIDX-10 < 1
        changeIDX = 11;
    end
    
    % Get a pool of 21 neighbouring points to estimate a color
    pool = colorSort(changeIDX-10:changeIDX+10,1:4);
    if isempty(pool(pool(:,4) ~= 0,:))
        warning("Couldn't determine point colors from neighbouring points")
        pointColors(change(p),:) = [0 0 0 0];
    else
        pointColors(change(p),:) = uint8([round(mean(pool(pool(:,4) ~= 0,1:3))) 0]);
    end
end

%% Plot whole point cloud
% f1 = figure;
% set(f1, 'Position', [400 50 1400 800])
% view([0 25])
% pcshow(pointCloud,pointColors(:,1:3));

% gif('SchlossV01.gif','DelayTime',1/40,'overwrite',true)
% for i = 0:360
%    view([i 25])
%    gif
% end

%% TODO:
% - Test with video instead of images?
end