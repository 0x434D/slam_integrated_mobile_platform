clear
close all
clc

%%

data_IP = table2array(readtable(".\Data\ScanRTKSecondRun.txt"));

%%
data_GNSS_Sim = simulateGNSS(data_IP,[-0.011 0.134 0.202]);
R =quat2rotm(data_GNSS_Sim(:,5:8));


% Initialize video
% myVideo = VideoWriter('offsetTestVid'); %open video file
% myVideo.FrameRate = 20;  %can adjust this, 5 - 10 works well for me
% open(myVideo)

figure(1)
hold on
% axis([-1 5 -1 7])
axis equal
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


for i=1800:10:8000 %size(data_GNSS_Sim,1)
    
    xQuat = [1 0; 0 0];
    yQuat = [0 0; 1 0];
    cart=[.8 -.8 -.8 .8; .5 .5 -.5 -.5];
    wheel1= [1.1 .5 .5 1.1; .4 .4 .75 .75];
    wheel2= [-1.1 -.5 -.5 -1.1; -.4 -.4 -.75 -.75];
    wheel3= [1.1 .5 .5 1.1; -.4 -.4 -.75 -.75];
    wheel4= [-1.1 -.5 -.5 -1.1; .4 .4 .75 .75];
    
    cart=[0 1; -1, 0] * R(1:2,1:2,i)*cart;
    cart=cart';
    cart=cart+data_IP(i,2:3);
    
    wheel1=[0 1; -1, 0] * R(1:2,1:2,i)*wheel1;
    wheel1=wheel1';
    wheel1=wheel1+data_IP(i,2:3);
    
    wheel2=[0 1; -1, 0] * R(1:2,1:2,i)*wheel2;
    wheel2=wheel2';
    wheel2=wheel2+data_IP(i,2:3);
    
    wheel3=[0 1; -1, 0] * R(1:2,1:2,i)*wheel3;
    wheel3=wheel3';
    wheel3=wheel3+data_IP(i,2:3);
    
    wheel4=[0 1; -1, 0] * R(1:2,1:2,i)*wheel4;
    wheel4=wheel4';
    wheel4=wheel4+data_IP(i,2:3);
    
    xQuat=[0 1; -1, 0] * R(1:2,1:2,i)*xQuat;
    xQuat=xQuat';
    xQuat=xQuat+data_IP(i,2:3);
    
    yQuat=[0 1; -1, 0] * R(1:2,1:2,i)*yQuat;
    yQuat=yQuat';
    yQuat=yQuat+data_IP(i,2:3);

    plot(data_IP(1700:i,2),data_IP(1700:i,3),'r-.')
    plot(data_GNSS_Sim(i,2), data_GNSS_Sim(i,3),'k.')

    refreshdata
    drawnow
    
    frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
end

% close(myVideo);