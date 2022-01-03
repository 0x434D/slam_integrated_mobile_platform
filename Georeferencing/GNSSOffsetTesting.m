    %% Main Code for simulateGNSS.m Testing


clear
close all
clc

%% Import scan data

data_IP = table2array(readtable(".\Data\ScanRTKSecondRun.txt"));

%% Call simulateGNSS

additionalRot = [0 0 1; -1 0 0; 0 -1 0];
dir = 2;


data_GNSS_Sim = simulateGNSS(data_IP,[-0.011 0.134 0.202], additionalRot, dir);

%% Rotation matrix from scanner quaternions
R =quat2rotm(data_IP(:,5:8));


%% Plot results
% Initialize video
% myVideo = VideoWriter('offsetTestVid'); %open video file
% myVideo.FrameRate = 20;  %can adjust this, 5 - 10 works well for me
% open(myVideo)


figure(1)
h1 = axes;
set (h1, 'Xdir', 'reverse');
set (h1, 'Ydir', 'reverse');
set(h1, 'XAxisLocation', 'Top')
set(h1, 'YAxisLocation', 'Right')
hold on
axis equal

% Car parts and local coordinate system axis
p_xQuat=plot(0,0,'-','Color','#77AC30','LineWidth',1.5);
p_xQuat.XDataSource = 'xQuat(:,1)';
p_xQuat.YDataSource = 'xQuat(:,2)';

p_yQuat=plot(0,0,'-','Color','#EDB120','LineWidth',1.5);
p_yQuat.XDataSource = 'yQuat(:,1)';
p_yQuat.YDataSource = 'yQuat(:,2)';

p_cart=plot(0,0,'-k','LineWidth',3);
p_cart.XDataSource = 'cart([1:4 1],1)';
p_cart.YDataSource = 'cart([1:4 1],2)';

p_wheel1=plot(0,0,'-k','LineWidth',3);
p_wheel1.XDataSource = 'wheel1([1:4 1],1)';
p_wheel1.YDataSource = 'wheel1([1:4 1],2)';

p_wheel2=plot(0,0,'-k','LineWidth',3);
p_wheel2.XDataSource = 'wheel2([1:4 1],1)';
p_wheel2.YDataSource = 'wheel2([1:4 1],2)';

p_wheel3=plot(0,0,'-k','LineWidth',3);
p_wheel3.XDataSource = 'wheel3([1:4 1],1)';
p_wheel3.YDataSource = 'wheel3([1:4 1],2)';

p_wheel4=plot(0,0,'-k','LineWidth',3);
p_wheel4.XDataSource = 'wheel4([1:4 1],1)';
p_wheel4.YDataSource = 'wheel4([1:4 1],2)';


% Start from 1800 because first 1800 points are just turning on the same spot
for i=1800:10:8000 %size(data_GNSS_Sim,1)
    
    % Set car part and axis coordinates
    xQuat = [1 0; 0 0; 0 0];
    yQuat = [0 0; 1 0; 0 0];
    cart=[.5 .5 -.5 -.5; .8 -.8 -.8 .8];
    wheel1= [ .4 .4 .75 .75; 1.1 .5 .5 1.1];
    wheel2= [-.4 -.4 -.75 -.75; -1.1 -.5 -.5 -1.1];
    wheel3= [-.4 -.4 -.75 -.75; 1.1 .5 .5 1.1];
    wheel4= [.4 .4 .75 .75; -1.1 -.5 -.5 -1.1];
    
    switch dir
        case 1
            RotMat = additionalRot * R(:,:,i);
            
        case 2
            RotMat = R(:,:,i) * additionalRot;
            
    end
   

    cart= RotMat(1:2,1:2) * cart;
    cart=cart';
    cart=cart+data_IP(i,2:3);

    wheel1= RotMat(1:2,1:2) * wheel1;
    wheel1=wheel1';
    wheel1=wheel1+data_IP(i,2:3);

    wheel2= RotMat(1:2,1:2) * wheel2;
    wheel2=wheel2';
    wheel2=wheel2+data_IP(i,2:3);

    wheel3= RotMat(1:2,1:2) * wheel3;
    wheel3=wheel3';
    wheel3=wheel3+data_IP(i,2:3);

    wheel4= RotMat(1:2,1:2) * wheel4;
    wheel4=wheel4';
    wheel4=wheel4+data_IP(i,2:3);

    xQuat= RotMat * xQuat;
    xQuat=xQuat(1:2,1:2)';
    xQuat=xQuat+data_IP(i,2:3);

    yQuat= RotMat * yQuat;
    yQuat=yQuat(1:2,1:2)';
    yQuat=yQuat+data_IP(i,2:3);
            
    % Plot trajectories
    plot(data_IP(1800:i,2),data_IP(1800:i,3),'r-.')
    plot(data_GNSS_Sim(i,2), data_GNSS_Sim(i,3),'k.')

    % Update car part and axis plots
    refreshdata
    drawnow
    
    % Write frame to video
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
end

% close(myVideo);