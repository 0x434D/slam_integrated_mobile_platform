%% Colour coding point cloud
% Author: SIMP Team
clearvars
close all
format longG
clc
fprintf('Color coding point cloud\n')

%% Read cloud data
lasReader = lasFileReader('D:\Johannes\Downloads\GeoreferencedPointcloud3Schloss.las');
[ptCloud,pointTimes] = readPointCloud(lasReader,'Attributes','GPSTimeStamp');
% pointTimes = seconds(pointTimes.GPSTimeStamp-seconds(1642604666.37918)+seconds(63809913318));     % Stadtgarten  % MatLab time in [s]
pointTimes = seconds(pointTimes.GPSTimeStamp-seconds(1643105216.7643)+seconds(63810413860.8));   % Schloss

% original ScanBackup time(1): 1642604666.37918 [s]
%                     Schloss: 1643105216.7643
% time offset from Georeferencing: 63809913318 [s]
%                         Schloss: 63810413860.8 
% figure
% pcshow(ptCloud.Location,ptCloud.Color)

%% Get image data
% % Read video and save dimensions 
% vidObj = VideoReader('360test.mp4');
% vid = read(vidObj);                     % format [x y rgb frame]
% vidDim = size(vid);                     % save size of format

% % Get the corresponding time stamp of frames [seconds]
% frameTimes = (1:vidDim(4))'.*vidObj.Duration/vidDim(4);

% Define folder and create image list
imgPath = 'D:\Johannes\Downloads\GoPro25';
addpath(imgPath)
imgList = ls(imgPath);
imgList = imgList(3:end,:);             % first two entries are not files

% Read images (and dimensions, times) in a loop
dsFactor = 3;   % downsample factor for images (reduce memory)
iDim = [2880/dsFactor 5760/dsFactor 3 size(imgList,1)];	% save size of image array
img = zeros(iDim(1),iDim(2),iDim(3),iDim(4),'uint8');  	% format [x y rgb frame]
for i = 1:size(imgList,1)
    fullImg = imread(imgList(i,:));
    img(:,:,:,i) = fullImg(1:dsFactor:end,1:dsFactor:end,:);
    frameDates(i,:) = datetime(imfinfo(imgList(i,:)).DateTime,'InputFormat','yyyy:MM:dd HH:mm:ss');
    frameTimes(i,1) = datenum(frameDates(i,:))*24*3600;	% MatLab time in [s]
end

%% Connect image and scan data via timestamps
% Loop over all images to find the closest scan-point index for each frame
for i = 1:length(frameTimes)
    [~,idx] = min(abs(pointTimes-frameTimes(i)));
    pointIdx(i,1) = idx;
end

% Define the scan points for each image with indices of bounds
% First image: [bounds(1) bounds(2)], Second: [bounds(2) bounds(3)] ...
idxBounds = [1;pointIdx(1:end-1)+floor(diff(pointIdx)/2);length(pointTimes)];

%% Load GNSS trajectory from MatLab file (Georeferencing output)
gnss = load('GNSS_Trajectory_SchlossRun.mat').GNSS;
scan = load('Scan_Trajectory_SchlossRun.mat').Scan;
rotScale = load('Scan_Trajectory_SchlossRun.mat').rotScale;

% DELETE LATER (this is beacuse the pointcloud is also flipped)
gnss = [gnss(:,1) gnss(:,3) gnss(:,2) -gnss(:,4)];
scan = [scan(:,1) scan(:,3) scan(:,2) -scan(:,4) scan(:,5:8)];

%% Loop over all images
pointColors = uint8(zeros(size(pointTimes,1),3));
for i = 1:size(frameTimes,1)-1 
%% Get orientation and position of current frame
% Add experimental time offset to frame times because the actual recording 
% time is always a little earlier
% time = frameTimes(i);
time = frameTimes(i)-0.7;       

% Function to get current orientation from GNSS trajectory 
[orient2,camOrig] = tra2ori(gnss,time,rotScale,"GNSS");
[orient,camOrig2] = tra2ori(scan,time,rotScale,"SCAN");

% camOrig = camOrig + [0.03 0.03 0.05];

% Load corresponding scan points for image
points = ptCloud.Location(idxBounds(i):idxBounds(i+1),:);

%% Find rgb for each scan point
% Define origin and calculate the vector of each scan point to origin
camOrig = camOrig - [0 0 0.13];  % antenna position (13 cm below antenna)
diff = points-camOrig;

% Spheric coordinates (two angles) from cartestian coordinates
[th,phi,~] = cart2sph(diff(:,1),diff(:,2),diff(:,3));

% Adjust orientation with vehicle trajectory 
% (adding orient results in clockwise rotation)
th = th+pi-(3*pi/2-orient);     % this if scanner is at the sides
% th = th+pi-(pi/2-orient);     % this if scanner is in the middle

% Spheric angles to MatLab plot pixel positions
% (-1 and +1 to assure result to be withtin [1 1920] or [1 1080])
xImg = round(mod(th,2*pi)*(iDim(2)-1)/(2*pi))+1;
yImg = round((pi/2-phi)*(iDim(1)-1)/pi)+1;

% Extract RGB color of the pixel(s) from 4D picture matrix
one = ones(length(xImg),1);
rgb = [img(sub2ind(size(img), yImg, xImg, one*1, one*i))...
       img(sub2ind(size(img), yImg, xImg, one*2, one*i))...
       img(sub2ind(size(img), yImg, xImg, one*3, one*i))];
rgb = uint8(rgb);       % convert to uint8 for .las color

%% Showcase
% Show rotated image and current scan points to be colored
% close all
% figure
% % rotIdx = [round(orient/(2*pi)*iDim(2)):iDim(2) 1:round(orient/(2*pi)*iDim(2))-1]';
% % imshow(img(:,rotIdx,:,i))   
% imshow(img(:,:,:,i))
% hold on
% scatter(xImg,yImg,7,'b','filled')
% drawnow
% % pause(0.5)

% figure
% pcshow(ptCloud.Location(idxBounds(i):idxBounds(i+1),:),rgb)

% Add up detected point colors for all images
pointColors(idxBounds(i):idxBounds(i+1),:) = rgb;

% % Pick one image fo showcase
% if i == 276
%     figure 
%     imshow(img(:,:,:,i))
%     hold on
%     scatter(xImg,yImg,7,'b','filled')
%     drawnow
%     
%     figure
%     pcshow(ptCloud.Location(idxBounds(i):idxBounds(i+1),:),rgb)
%     i
% end
end

%% Plot whole point cloud
f1 = figure;
pcshow(ptCloud.Location,pointColors)

% set(f1, 'Position', [400 50 1400 800])
% view([0 25])
% gif('SchlossV01.gif','DelayTime',1/40,'overwrite',true)
% for i = 0:360
%    view([i 25])
%    gif
% end

%% TODO:
% - Get image orientation from scanning trajectory
%       - Load GPS time in useful format (year, month, day etc.)
%       - important that first image is after movement started, otherwise
%       the orientation doesn't work --> applied quick fix but not perfect.
%       The first and last picture should be handpicked
% - Scan coordinates should be in flat earth frame
% - Try changing image time a little, maybe the saved time is always a few
% 0.1 seconds earlier
% - Create a mask for Scanner and antenna platform
% - Test with video instead of images?