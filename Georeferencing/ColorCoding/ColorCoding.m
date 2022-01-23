%% Colour coding point cloud
% Author: SIMP Team
clearvars
% close all
format longG
clc
fprintf('Color coding point cloud\n')

%% Read cloud data
lasReader = lasFileReader('D:\Johannes\Downloads\GeoreferencedPointcloud3.las');
[ptCloud,pointTimes] = readPointCloud(lasReader,'Attributes','GPSTimeStamp');
pointTimes = seconds(pointTimes.GPSTimeStamp-seconds(1642604666.37918)+seconds(63809913318));
% epoch = datetime(1970,1,1,'TimeZone','UTCLeapSeconds');         % UNIX time start
% pointTimes = datenum(epoch + pointTimes.GPSTimeStamp-seconds(1642604666.37918))*24*3600;  % MatLab time in [s]
% datetime(stamps(1)/(24*3600),'ConvertFrom','datenum','Format', 'yyyy-MM-dd HH:mm:ss.SSS')

% original ScanBackup time(1): 1642604666.37918 [s]
% time offset from Georeferencing: 63809913318 [s]
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
imgPath = 'D:\Johannes\Downloads\GoPro19_1';
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
% Pseude image times (only for testing, remove in final program)
% frameTimes = [pointTimes(10000);pointTimes(230000);pointTimes(560000);pointTimes(1000000);pointTimes(4350000);pointTimes(8000000)];
% frameTimes = linspace(pointTimes(1),pointTimes(end),size(imgList,1))';

% Loop over all images to find the closest scan-point index for each frame
for i = 1:length(frameTimes)
    [~,idx] = min(abs(pointTimes-frameTimes(i)));
    pointIdx(i,1) = idx;
end

% Define the scan points for each image with indices of bounds
% First image: [bounds(1) bounds(2)], Second: [bounds(2) bounds(3)] ...
idxBounds = [1;pointIdx(1:end-1)+floor(diff(pointIdx)/2);length(pointTimes)];

%% Load GNSS trajectory from MatLab file (Georeferencing output)
gnss = load('GNSS_Trajectory_RTKrun.mat').GNSS;
scan = load('Scan_Trajectory_RTKrun.mat').Scan;
rotScale = load('Scan_Trajectory_RTKrun.mat').rotScale;

% DELETE LATER (this is beacuse the pointcloud is also flipped)
gnss = [gnss(:,1) gnss(:,3) gnss(:,2) -gnss(:,4)];
scan = [scan(:,1) scan(:,3) scan(:,2) -scan(:,4) scan(:,5:8)];

%% Loop over all images
pointColors = uint8(zeros(size(pointTimes,1),3));
for i = 1:size(frameTimes,1)-1
% time = gnss(100,1)-0.1;
time = frameTimes(i);

% Function to get current orientation from GNSS trajectory 
% [orient,camOrig] = tra2ori(gnss,time,rotScale);
[orient,camOrig] = tra2ori(scan,time,rotScale);

% % Plot trajectories
% figure
% pGNSS = plot3(gnss(:,2),gnss(:,3),gnss(:,4),'b','LineWidth',1);
% hold on
% view([0 90])
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% title('Showcase of orientation from GNSS trajectory')
% text(-70,-32,-254,'Azimuth:')
% tempText = text(-70,-37,-254,"0°");
% % gif('OriFromGNSS.gif','DelayTime',1/40,'overwrite',true)
% % for i = 10:size(gnss,1)
% for i = 1:size(frameTimes,1)
% %     time = gnss(i,1)+0.1;
%     time = frameTimes(i);
%     orient = tra2ori(gnss,time);
%     delete(tempText)
%     tempText = text(-70,-37,-254,num2str(round(orient*180/pi)) + "°");
%     pause(0.5)
% %     gif
% %     plot3(gnss(idxLB:idxUB,2),gnss(idxLB:idxUB,3),gnss(idxLB:idxUB,4),'r','LineWidth',2)
% %     drawnow
% end

%% Map 360° image data to a sphere
% % Map colors to unit sphere
% az = linspace(0,2*pi,iDim(2));
% el = linspace(pi/2,-pi/2,iDim(1))';
% [azGrid,elGrid] = meshgrid(az,el);
% azi = reshape(azGrid,iDim(1)*iDim(2),1);
% ele = reshape(elGrid,iDim(1)*iDim(2),1);
% rad = ones(iDim(1)*iDim(2),1);
% red = double(reshape(img(:,:,1),iDim(1)*iDim(2),1));
% green = double(reshape(img(:,:,2),iDim(1)*iDim(2),1));
% blue = double(reshape(img(:,:,3),iDim(1)*iDim(2),1));
% 
% [X,Y,Z] = sph2cart(azi,ele,rad);
% colorCloud = [X,Y,Z,red,green,blue];
% ccSub = colorCloud(1:10:end,:);

%% Create test scan points
% numP = 1;
% wall1x = ones(numP^2,1)*-3;                                   	% Wall 1
% wall1y = linspace(-4,4,numP);
% wall1z = linspace(-2,2,numP);
% [Ywall,Zwall] = meshgrid(wall1y,wall1z);
% wall1 = [wall1x reshape(Ywall,[numP^2,1]) reshape(Zwall,[numP^2,1])];
% 
% wall2y = ones(numP^2,1)*-4;                                  	% Wall 2
% wall2x = linspace(-3,3,numP);
% wall2z = linspace(-2,2,numP);
% [Xwall,Zwall] = meshgrid(wall2x,wall2z);
% wall2 = [reshape(Xwall,[numP^2,1]) wall2y reshape(Zwall,[numP^2,1])];
% 
% wall3x = ones(numP^2,1)*3;                                     	% Wall 3
% wall3y = linspace(-4,4,numP);
% wall3z = linspace(-2,2,numP);
% [Ywall,Zwall] = meshgrid(wall3y,wall3z);
% wall3 = [wall3x reshape(Ywall,[numP^2,1]) reshape(Zwall,[numP^2,1])];
% 
% wall4y = ones(numP^2,1)*4;                                    	% Wall 4
% wall4x = linspace(-3,3,numP);
% wall4z = linspace(-2,2,numP);
% [Xwall,Zwall] = meshgrid(wall4x,wall4z);
% wall4 = [reshape(Xwall,[numP^2,1]) wall4y reshape(Zwall,[numP^2,1])];
% 
% wall5z = ones(numP^2,1)*2;                                     	% Roof
% wall5x = linspace(-3,3,numP);
% wall5y = linspace(-4,4,numP);
% [Xwall,Ywall] = meshgrid(wall5x,wall5y);
% wall5 = [reshape(Xwall,[numP^2,1]) reshape(Ywall,[numP^2,1]) wall5z];
% 
% wall6z = ones(numP^2,1)*-2;                                     % Floor
% wall6x = linspace(-3,3,numP);
% wall6y = linspace(-4,4,numP);
% [Xwall,Ywall] = meshgrid(wall6x,wall6y);
% wall6 = [reshape(Xwall,[numP^2,1]) reshape(Ywall,[numP^2,1]) wall6z];
% 
% % walls = [wall1;wall2;wall3;wall4;wall5;wall6];
walls = ptCloud.Location(idxBounds(i):idxBounds(i+1),:);
% walls = single([0 0 250]);

%% Find rgb for each scan point
% Define origin and calculate the vector of each scan point to origin
% camOrig = [0 0 1];
camOrig = camOrig - [0 0 0.12];  % antenna position (12 cm below antenna)
diff = walls-camOrig;
% diff = [0 10 0; 10 0 0];

% Spheric coordinates (two angles) from cartestian coordinates
[th,phi,~] = cart2sph(diff(:,1),diff(:,2),diff(:,3));

% Adjust orientation with vehicle trajectory 
% (adding orient results in clockwise rotation)
% orient = 0;
% th = th+orient;
% th = 2*pi-orient+th;
th = th+pi-(pi/2-orient);


% Spheric angles to MatLab plot pixel positions
% (-1 and +1 to assure result to be withtin [1 1920] or [1 1080])
xImg = round(mod(th,2*pi)*(iDim(2)-1)/(2*pi))+1;
yImg = round((pi/2-phi)*(iDim(1)-1)/pi)+1;

% Extract RGB color of the pixel(s) from 4D picture matrix
one = ones(length(xImg),1);
rgb = [img(sub2ind(size(img), yImg, xImg, one*1, one))...
       img(sub2ind(size(img), yImg, xImg, one*2, one))...
       img(sub2ind(size(img), yImg, xImg, one*3, one))];
% rgb = double(rgb)/256;  % convert to double and normalize as MatLab color
rgb = uint8(rgb);       % convert to uint8 for .las color

% Add rgb colors to point matrix
% walls = [walls rgb];

%% Showcase
% Show rotated image and current scan points to be colored
% % close all
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
i;
end
%%
figure
pcshow(ptCloud.Location,pointColors)

% figure
% scatter3(ccSub(:,1),ccSub(:,2),ccSub(:,3),5,[double(ccSub(:,4))/256 double(ccSub(:,5))/256 double(ccSub(:,6))/256],'filled')
% hold on
% scatter3(walls(:,1), walls(:,2), walls(:,3),10,[walls(:,4) walls(:,5) walls(:,6)],'filled')
% % plot3([0 walls(1,1)],[0 walls(1,2)],[0 walls(1,3)],'Color',[walls(1,4) walls(1,5) walls(1,6)],'LineWidth',1.5)
% axis equal
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% view([0 25])
% % % gif('wallPointsGif.gif','DelayTime',1/40,'overwrite',true)
% % % for i = 1:0.5:360
% % %     view([i 5+(i*0.2)])
% % %     gif
% % % end

%% TODO:
% - Test with images instead of video (what if image is captured with angled gopro?)
%       - image becomes angled
% - Inlcude times from images? --> Only accuracy of seconds (should suffice)
% - Get image orientation from scanning trajectory
%       - Load GPS time in useful format (year, month, day etc.)
%       - important that first image is after movement started, otherwise
%       the orientation doesn't work --> applied quick fix but not perfect.
%       The first and last picture should be handpicked
% - Scan coordinates should be in flat earth frame
% - Get all scan points for each image (stack together with ids) and
%   calculate colors for each point
%       - Make loop over all images
% - How to use uniform time in all scripts? Datenum is in days, so we
%   mutliply that with 24*3600
% - How to get the right epoch for the scan points? Must be determined from
%   scanner somehow
% - Improve matching points into image (orient, camer pos usw. , fix the 
%   problem with flat earth rotation) --> ALso go through frame by frame
%   and check the matching of position and scan points
% - ISt phi richtig aufgeteilt?
% - Include Scanner Trajectory
% - Try changing image time a little, maybe the saved time is always a few
% 0.1 seconds earlier
% - NEXT: Use GNSS pos instead of SCAN pos but with SCAN orientation