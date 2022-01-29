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
mask = logical([zeros(150,1920);ones(660,1920);zeros(150,1920)]);

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
pointColors = uint8(zeros(size(pointTimes,1),3));
for i = 1:size(frameTimes,1)-1
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
pointColors(idxBounds(i):idxBounds(i+1),:) = rgb;

%% Showcase
% % Show rotated image (pick with i) and current scan points to be colored
% if i == 31
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

%% Plot whole point cloud
f1 = figure;
set(f1, 'Position', [400 50 1400 800])
view([0 25])
pcshow(pointCloud,pointColors);

% gif('SchlossV01.gif','DelayTime',1/40,'overwrite',true)
% for i = 0:360
%    view([i 25])
%    gif
% end

%% TODO:
% - Scan coordinates should be in flat earth frame
% - Create a mask for Scanner and antenna platform
% - Test with video instead of images?
end

