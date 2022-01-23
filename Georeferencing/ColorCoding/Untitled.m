clear all 
% close all

scan = load('Scan_Trajectory_RTKrun.mat').Scan;
rotScale = load('Scan_Trajectory_RTKrun.mat').rotScale;

scan = [scan(:,1) scan(:,3) scan(:,2) -scan(:,4) scan(:,5:8)];

figure
plot3(scan(:,2),scan(:,3),scan(:,4))
hold on
grid on
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
view([0 90])

stick = [0 0 0; 0 2 0];
% 902 corresponds to image 5
for idx = 902:20:length(scan)
    % Directly get the azimuth from quaternions
    DCM11 = scan(idx,5)^2+scan(idx,6)^2-scan(idx,7)^2-scan(idx,8)^2;
    DCM21 = 2*(scan(idx,6)*scan(idx,7)-scan(idx,8)*scan(idx,5));
    orient = mod(atan2(DCM21,DCM11),2*pi);
%     orient = mod(atan2(DCM11,DCM21),2*pi);

    rotMat = rotScale*quat2rotm(scan(idx,5:8))*[0 0 1; -1 0 0; 0 -1 0];
    orient = 2*pi-mod(atan2(rotMat(1,2),rotMat(1,1))+pi/2,2*pi);
    orient*180/pi
    stickNew = stick*[cos(orient) -sin(orient) 0; sin(orient) cos(orient) 0; 0 0 1];
%     stickNew = (rotScale*(rotMat*stick'))';
    
    stickNew = stickNew + scan(idx,2:4);
    
    stickPlot = plot3(stickNew(:,1),stickNew(:,2),stickNew(:,3),'k','LineWidth',2);
    stickHead = scatter3(stickNew(2,1),stickNew(2,2),stickNew(2,3),10,'r','filled');
    drawnow
    pause(0.01)
    idx
    delete(stickPlot);
end